"""PostToolUse hook: regenerate the context atlas after Claude edits a repo file."""
from pathlib import Path
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ATLAS = ROOT / 'docs' / 'context-atlas'
GENERATOR = ROOT / 'scripts' / 'generate_context_index.py'
SUFFIXES = {'.java', '.gradle', '.xml', '.md', '.json', '.py', '.yml', '.yaml', '.properties'}


def main():
    try:
        event = json.load(sys.stdin)
        target = (event.get('tool_input') or {}).get('file_path') or (event.get('tool_input') or {}).get('notebook_path')
        if not target:
            return 0
        path = Path(target).resolve()
        path.relative_to(ROOT)
    except Exception:
        return 0
    if ATLAS in path.parents or path.suffix.lower() not in SUFFIXES:
        return 0
    result = subprocess.run([sys.executable, '-I', str(GENERATOR)], cwd=ROOT, capture_output=True, text=True, timeout=60)
    if result.returncode != 0:
        sys.stderr.write('Context atlas refresh failed:\n' + (result.stderr or result.stdout)[-800:])
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
