"""Stdio MCP server over docs/context-atlas/index.json. Standard library only."""
from pathlib import Path
import hashlib
import json
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ATLAS = ROOT / 'docs' / 'context-atlas'
INDEX = ATLAS / 'index.json'
INDEX_MD = ATLAS / 'INDEX.md'
GENERATOR = ROOT / 'scripts' / 'generate_context_index.py'
TEAM = 'TeamCode/'
SERVER_VERSION = '1.0.0'

_cache = {'mtime': None, 'data': None, 'dependents': None}


def load():
    mtime = INDEX.stat().st_mtime_ns
    if _cache['mtime'] != mtime:
        data = json.loads(INDEX.read_text(encoding='utf-8'))
        dependents = {}
        for edge in data['edges']:
            dependents.setdefault(edge['to'], []).append(edge['from'])
        _cache.update(mtime=mtime, data=data, dependents=dependents)
    return _cache['data'], _cache['dependents']


def current_head():
    try:
        return subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    except Exception:
        return 'unknown'


def disk_sha(path):
    file = ROOT / path
    return hashlib.sha256(file.read_bytes()).hexdigest() if file.is_file() else None


def stale_files(data):
    stale = []
    for j in data['java_files']:
        if j['path'].startswith(TEAM) and disk_sha(j['path']) != j['sha256']:
            stale.append(j['path'])
    return stale


def md_section(title):
    text = INDEX_MD.read_text(encoding='utf-8')
    match = re.search(r'^## ' + re.escape(title) + r'\n(.*?)(?=^## |\Z)', text, re.S | re.M)
    return match.group(1).strip() if match else ''


def tool_overview(_args):
    data, _ = load()
    stale = stale_files(data)
    out = [
        f"Atlas generated {data['generated_date']} at {data['git_head'][:7]}; repo HEAD is {current_head()[:7]}.",
        f"Stale TeamCode files (changed on disk since indexing): {len(stale)}"
        + (' -> ' + ', '.join(Path(p).name for p in stale[:10]) if stale else '') + '.',
        '', '## Architecture', md_section('Architecture'),
        '', '## Current source facts', md_section('Current source facts'),
        '', '## Reading routes', md_section('Reading routes'),
    ]
    return '\n'.join(out)


def resolve(data, query):
    query = query.replace('\\', '/').strip()
    files = data['java_files']
    exact = [j for j in files if j['path'] == query]
    if exact:
        return exact
    suffix = [j for j in files if j['path'].endswith('/' + query) or j['path'].endswith(query)]
    if suffix:
        return suffix
    stem = Path(query).stem.lower()
    return [j for j in files if Path(j['path']).stem.lower() == stem]


def tool_find(args):
    data, _ = load()
    tokens = [t for t in re.split(r'[\s,]+', args.get('query', '').lower()) if t]
    if not tokens:
        return 'Give a query: a class, method, file or topic word.'
    include_sdk = bool(args.get('include_sdk', False))
    limit = int(args.get('limit', 12))
    ranked = []
    for j in data['java_files']:
        if not include_sdk and not j['path'].startswith(TEAM):
            continue
        stem = Path(j['path']).stem.lower()
        score = 0
        hits = []
        for t in tokens:
            if stem == t:
                score += 10
            elif t in stem:
                score += 5
            if t in j['path'].lower():
                score += 2
            if any(t in ty['name'].lower() for ty in j['types']):
                score += 6
            if t in j['role'].lower():
                score += 1
            for m in j['methods']:
                if t in m['name'].lower():
                    score += 3
                    hits.append(m)
        if score:
            ranked.append((score, j, hits))
    ranked.sort(key=lambda r: -r[0])
    if not ranked:
        return 'No atlas match. Fall back to Grep, then run atlas_refresh if the file is new.'
    lines = []
    for score, j, hits in ranked[:limit]:
        short = j['path'].replace('TeamCode/src/main/java/org/firstinspires/ftc/teamcode/', '')
        lines.append(f"{short}  ({j['path']}, {j['lines']} lines)\n  {j['role']}")
        seen = set()
        for m in hits:
            if m['signature'] not in seen and len(seen) < 6:
                seen.add(m['signature'])
                lines.append(f"  L{m['line']}: {m['signature']}")
    return '\n'.join(lines)


def tool_file(args):
    data, dependents = load()
    matches = resolve(data, args.get('path', ''))
    if not matches:
        return 'Not in the atlas. It may be new (run atlas_refresh) or not a Java file.'
    out = []
    for j in matches[:3]:
        live = disk_sha(j['path'])
        state = 'missing on disk' if live is None else ('current' if live == j['sha256'] else 'STALE: changed since indexing, re-read the file or run atlas_refresh')
        out.append(f"{j['path']}\nRole: {j['role']}\nIndexed lines: {j['lines']}   Atlas state: {state}")
        out.append('Types: ' + ', '.join(f"{t['name']} ({t['kind']}, L{t['line']})" for t in j['types']))
        out.append('Methods:\n' + '\n'.join(f"  L{m['line']}: {m['signature']}" for m in j['methods']))
        out.append('Depends on:\n' + '\n'.join('  ' + p for p in j['internal_dependencies']) if j['internal_dependencies'] else 'Depends on: none')
        deps = sorted(dependents.get(j['path'], []))
        out.append('Used by:\n' + '\n'.join('  ' + p for p in deps) if deps else 'Used by: none')
        out.append('')
    return '\n'.join(out)


def tool_refresh(_args):
    result = subprocess.run([sys.executable, '-I', str(GENERATOR)], cwd=ROOT, capture_output=True, text=True, timeout=120)
    if result.returncode != 0:
        return 'Refresh failed:\n' + (result.stderr or result.stdout)[-1500:]
    data, _ = load()
    return f"Refreshed. Now at {data['git_head'][:7]}; stale TeamCode files: {len(stale_files(data))}.\n{result.stdout.strip()}"


TOOLS = {
    'atlas_overview': (tool_overview, 'Project architecture, current source facts, reading routes and index freshness. Call this first in a session.', {}),
    'atlas_find': (tool_find, 'Find files, classes and methods by name or topic. Returns paths, roles and method signatures with line numbers, so you can Read only the lines you need.',
                   {'query': {'type': 'string', 'description': 'Class, method, file or topic words, e.g. "pauseFollowing" or "turret limits".'},
                    'include_sdk': {'type': 'boolean', 'description': 'Also search FTC SDK/vendor files. Default false.'},
                    'limit': {'type': 'integer', 'description': 'Max files. Default 12.'}}),
    'atlas_file': (tool_file, 'Full outline of one file: role, types, methods with line numbers, dependencies, dependents, and whether the atlas copy is stale.',
                   {'path': {'type': 'string', 'description': 'Repo-relative path, file name, or class name, e.g. "Robot.java".'}}),
    'atlas_refresh': (tool_refresh, 'Regenerate the atlas now. Normally automatic after edits through the PostToolUse hook.', {}),
}
REQUIRED = {'atlas_find': ['query'], 'atlas_file': ['path']}


def tool_list():
    return [{'name': n, 'description': d,
             'inputSchema': {'type': 'object', 'properties': p, 'required': REQUIRED.get(n, [])}}
            for n, (_, d, p) in TOOLS.items()]


def handle(msg):
    method = msg.get('method')
    mid = msg.get('id')
    if mid is None:
        return None
    if method == 'initialize':
        version = (msg.get('params') or {}).get('protocolVersion', '2024-11-05')
        result = {'protocolVersion': version, 'capabilities': {'tools': {}},
                  'serverInfo': {'name': 'context-atlas', 'version': SERVER_VERSION}}
    elif method == 'tools/list':
        result = {'tools': tool_list()}
    elif method == 'tools/call':
        params = msg.get('params') or {}
        name = params.get('name')
        if name not in TOOLS:
            return {'jsonrpc': '2.0', 'id': mid, 'error': {'code': -32602, 'message': 'Unknown tool: ' + str(name)}}
        try:
            text, error = TOOLS[name][0](params.get('arguments') or {}), False
        except Exception as exc:
            text, error = f'{type(exc).__name__}: {exc}', True
        result = {'content': [{'type': 'text', 'text': text}], 'isError': error}
    elif method == 'ping':
        result = {}
    else:
        return {'jsonrpc': '2.0', 'id': mid, 'error': {'code': -32601, 'message': 'Method not found: ' + str(method)}}
    return {'jsonrpc': '2.0', 'id': mid, 'result': result}


def main():
    for line in sys.stdin.buffer:
        line = line.strip()
        if not line:
            continue
        try:
            reply = handle(json.loads(line.decode('utf-8')))
        except Exception as exc:
            reply = {'jsonrpc': '2.0', 'id': None, 'error': {'code': -32700, 'message': str(exc)}}
        if reply is not None:
            sys.stdout.buffer.write((json.dumps(reply) + '\n').encode('utf-8'))
            sys.stdout.buffer.flush()


if __name__ == '__main__':
    main()
