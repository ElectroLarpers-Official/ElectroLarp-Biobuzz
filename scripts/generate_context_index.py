"""Generate local architecture docs from tracked files; no uploads or build execution."""
from pathlib import Path
from collections import Counter, defaultdict
import hashlib
import html
import json
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs' / 'context-atlas'
BASE = 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/'
STAMP = '2026-11-01'
ROLES = {
    'Robot': 'Owns hardware and follower; updates aiming and flywheel; coordinates shooting, manual fallback and pose reset.',
    'HardwareNames': 'Central robot configuration device names.',
    'Close': 'Near-side autonomous sequence: shooting, pollen loading, flower approach and retreat.',
    'Far': 'Far-side autonomous lifecycle; routine currently a placeholder.',
    'ClosePaths': 'Near-side paths, heading interpolation, mirrored poses and flower pose callback.',
    'FarPaths': 'Far-side pose factory and placeholder starting pose; paths not implemented.',
    'Tele': 'Two-driver controls, field-centric drive, shooting gates, manual presets and live trims.',
    'Intake': 'Intake plus optional belted transfer motor, controlled together.',
    'Shooter': 'Dual flywheel motors, encoder feedback, PIDF and optional shooting bang-bang control.',
    'Turret': 'Turret servo hardware, optional four-servo operation and robot-to-turret geometry.',
    'Hood': 'Hood servo position and launch-angle control.',
    'Latch': 'Optional shooting gate servo.',
    'Flower': 'Optional flower mechanism servo.',
    'ShooterCalc': 'Selects and solves shots; tuned-point/map lookup, motion prediction and hardware target conversion.',
    'ShooterConstants': 'Field/HIVE geometry, target selection constants and under-HIVE behavior.',
    'ShotCheck': 'Shot feasibility, shooting permission and readiness tolerances.',
    'Braking': 'Stopping prediction used by aiming.',
    'TurretCalc': 'Turret servo gearing, angular limits, slew, alliance offsets and yaw compensation.',
    'HiveAimTable': 'Queries simulated shot-map data by cell and field position.',
    'HiveAimTableData': 'Generated simulated shot-map data; use the generator workflow rather than editing values manually.',
    'TunedShotTable': 'Blends hand-tuned points; merges committed points with Control Hub shotpoints.csv; mirrors between cells.',
    'TunedShotPoints': 'Committed calibrated shot points.',
    'Constants': 'Builds PulseFollower from Pinpoint localization, Mecanum or NoDrivetrain, and Foresight configuration.',
    'NoDrivetrain': 'Drivetrain fallback when the four drive motors are absent.',
    'Tuning': 'Registers AutoTune procedures: Mecanum, Pinpoint, Foresight and Tests.',
    'CommandOpMode': 'LinearOpMode lifecycle driving the Ivy scheduler; resets scheduler before final end().',
    'TeleTelemetry': 'Driver Station and Panels telemetry presentation.',
    'OpModeStorage': 'Pose handoff between OpModes.',
    'Alliance': 'Shared alliance selection.',
    'CachedMotor': 'Motor hardware wrapper with cached writes.',
    'PIDController': 'Reusable PID feedback math.',
    'GamepadEx': 'Gamepad inputs with edge-aware buttons and suppliers.',
    'BooleanSwitch': 'Boolean input state and edge tracking.',
    'FloatSupplier': 'Analog input supplier and trigger helpers.',
    'Filter': 'Numeric filter contract.',
    'ScalingLowPassFilter': 'Scaling low-pass signal filtering.',
    'Tuple': 'Two-value data container.',
    'TriTuple': 'Three-value data container.',
    'FlywheelPIDFTest': 'Bench OpMode for intake, flywheel PIDF and solver-driven hood targets.',
    'ShotPointTuner': 'Drive, tune, save and export calibrated shot points.',
    'HoodServoPositionTuning': 'Hood calibration OpMode.',
    'TurretTrackingTest': 'Checks turret tracking using localization.',
    'TurretTest': 'Turret hardware test OpMode.',
    'TurretCenter': 'Turret centering OpMode.',
}


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True, encoding='utf-8').strip()


def no_comments(source):
    # Keep strings and line breaks, so references retain original source line numbers.
    token = r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*[\s\S]*?\*/'
    return re.sub(token, lambda m: re.sub(r'[^\n]', ' ', m[0]) if m[0].startswith(('/', '/*')) else m[0], source)


def safe_path(path):
    parts = path.lower().split('/')
    return not (any(x in parts for x in ('.git', '.idea', '.gradle', '.aws', '.codex', '.agents', 'build'))
                or any(x in parts[-1] for x in ('secret', 'credential', 'keystore', 'local.properties', '.env', '.jks', '.p12')))


def role(path):
    name = Path(path).stem
    if name in ROLES:
        return ROLES[name]
    if '/procedures/' in path:
        return 'AutoTune procedure: ' + name + '.'
    if '/auto/blue/' in path or '/auto/red/' in path:
        return 'Alliance-specific autonomous registration wrapper.'
    if name in ('teleRed', 'teleBlue'):
        return 'Alliance-specific TeleOp registration wrapper.'
    return 'FTC SDK/vendor source.' if path.startswith('FtcRobotController/') else 'Project support file.'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    paths = [p for p in git('ls-files').splitlines() if safe_path(p) and (ROOT / p).is_file()]
    java = []
    for path in paths:
        if not path.endswith('.java'):
            continue
        raw = (ROOT / path).read_text(encoding='utf-8-sig', errors='replace')
        code = no_comments(raw)
        pkg = re.search(r'\bpackage\s+([\w.]+)\s*;', code)
        types = [{'name': m[2], 'kind': m[1], 'line': code.count('\n', 0, m.start()) + 1}
                 for m in re.finditer(r'\b(class|interface|enum|record)\s+(\w+)', code)]
        methods = []
        pattern = r'\b(public|protected|private)\s+(?:(?:static|final|abstract|synchronized|native|default)\s+)*(?:<[^;{}]+>\s+)?(?:[\w.$<>\[\],?]+\s+)?(\w+)\s*\(([^;{}]*?)\)\s*(?:throws\s+[\w.,\s]+)?\s*(?=[{;])'
        for m in re.finditer(pattern, code):
            methods.append({'name': m[2], 'signature': ' '.join(m[0].split()), 'visibility': m[1],
                            'line': code.count('\n', 0, m.start()) + 1})
        imports = re.findall(r'\bimport\s+(?:static\s+)?([\w.*]+)\s*;', code)
        java.append({'path': path, 'package': pkg[1] if pkg else '', 'types': types, 'methods': methods,
                     'imports': imports, 'role': role(path), 'lines': len(raw.splitlines()),
                     'sha256': hashlib.sha256((ROOT / path).read_bytes()).hexdigest()})
    lookup = {j['package'] + '.' + Path(j['path']).stem: j['path'] for j in java}
    edges = []
    for j in java:
        targets = set()
        for imp in j['imports']:
            if imp.endswith('.*'):
                targets.update(p for fqn, p in lookup.items() if fqn.rsplit('.', 1)[0] == imp[:-2])
            else:
                name = imp
                while '.' in name:
                    if name in lookup:
                        targets.add(lookup[name])
                        break
                    name = name.rsplit('.', 1)[0]
        # Same-package type references are heuristic, not a resolved Java call graph.
        code = no_comments((ROOT / j['path']).read_text(encoding='utf-8-sig', errors='replace'))
        for other in java:
            if other['package'] == j['package'] and re.search(r'\b' + re.escape(Path(other['path']).stem) + r'\b', code):
                targets.add(other['path'])
        j['internal_dependencies'] = sorted(targets - {j['path']})
        edges.extend({'from': j['path'], 'to': target, 'kind': 'import-or-same-package-reference'} for target in j['internal_dependencies'])
    team = [j for j in java if j['path'].startswith(BASE)]
    package_groups = defaultdict(list)
    for j in team:
        package_groups[j['package']].append(j)
    dependencies = re.findall(r"implementation\s*\(?\s*['\"]([^'\"]+:[^'\"]+:[^'\"]+)['\"]", (ROOT / 'build.dependencies.gradle').read_text())
    snapshot = {'project': 'ElectroLarp-Biobuzz', 'generated_date': STAMP, 'git_head': git('rev-parse', 'HEAD'),
                'scope': 'Tracked non-sensitive repository paths; Java symbols and imports. No secrets, binary contents or generated build directories.',
                'limitations': 'Regex-based declarations and dependency candidates; not a compiler-resolved call graph. Commented-out declarations excluded. No runtime validation.',
                'inventory': [{'path': p, 'bytes': (ROOT / p).stat().st_size} for p in paths],
                'dependencies': dependencies, 'java_files': java, 'edges': edges}
    (OUT / 'index.json').write_text(json.dumps(snapshot, indent=2), encoding='utf-8')
    overview = f'''# ElectroLarp-Biobuzz project index

Generated {STAMP}, source HEAD `{snapshot['git_head']}`. Source files were inspected directly.

## Coverage

- {len(paths)} tracked non-sensitive paths inventoried.
- {len(team)} TeamCode Java files, {len(java) - len(team)} SDK/vendor Java files.
- {sum(len(j['methods']) for j in java)} method/constructor declarations; {len(edges)} candidate internal dependency edges.
- [Interactive mind map](mindmap.html), [Mermaid mind map](MINDMAP.md), [machine-readable inventory and symbols](index.json).
- [Regeneration script](../../scripts/generate_context_index.py): run `python scripts/generate_context_index.py` from this checkout. It writes local documents only; Atlas refresh is a separate explicit operation.

## Architecture

`redClose` / `blueClose` and `redFar` / `blueFar` select an alliance and inherit autonomous behavior.
`teleRed` / `teleBlue` select an alliance and inherit `Tele` controls.
`CommandOpMode` runs the Ivy scheduler inside the FTC LinearOpMode lifecycle.
`Robot` constructs the follower and six mechanisms, and owns coordinated shooting and aiming.
Each `Robot.update()` clears bulk caches, updates the follower, updates aiming, then updates the shooter.
`Constants.create()` builds a PulseFollower using Pinpoint localization, Foresight and Mecanum (or NoDrivetrain).
`ShooterCalc` selects tuned/table shots and computes targets; `TurretCalc` maps turret aim; `ShotCheck` gates shooting.
`OpModeStorage.endPose` carries the last pose into a later OpMode when enabled.

## Current source facts

- Intake and transfer run together in `Intake`; the transfer motor is optional. There is no separate Transfer class.
- Mechanism actions are ordinary methods in this snapshot. `Robot.shootCycle()` wraps startShooting / delay / stopShooting in Ivy commands.
- Close autonomous uses Ivy groups and electropathing follow/pose callbacks. No direct pulse-lumen dependency is declared.
- Far autonomous and its start pose are placeholders. Close and Far use different alliance-mirroring conventions; verify geometry before field use.
- Flower and latch are optional hardware. Turret supports the temporary four-servo configuration.
- TunedShotTable combines committed points with a Control Hub CSV; this index does not read that device file.
- Historical tuning prose can be stale: current `Tuning.java` registers four tuners despite an older section saying it is blank.

## Reading routes

| Task | Start here |
|---|---|
| Autonomous paths / flower callback | auto/Close.java, auto/paths/ClosePaths.java, subsystem/Flower.java |
| Transfer / intake behavior | subsystem/Intake.java, Robot.startShooting / stopShooting |
| Driver controls | tele/Tele.java, util/input/GamepadEx.java |
| Shot accuracy | subsystem/aim/shot/ShooterCalc.java, subsystem/aim/table/TunedShotTable.java |
| Flywheel control | subsystem/Shooter.java, util/control/PIDController.java, test/FlywheelPIDFTest.java |
| Turret geometry | subsystem/Turret.java, subsystem/aim/turret/TurretCalc.java |
| Localization / drive | pedro/Constants.java, pedro/Tuning.java, pedro/procedures/ |
| Pose handoff / lifecycle | util/CommandOpMode.java, util/OpModeStorage.java |
| Device configuration | HardwareNames.java, configs/LightningLM1.xml |
| Team workflow | docs/git.md, .githooks/commit-msg, .github/workflows/ |

Paths in this table are relative to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`, except build/config/workflow paths.

## Direct dependencies

''' + '\n'.join('- `' + d + '`' for d in dependencies) + '''

## Build and repository structure

- `TeamCode/`: robot behavior, mechanisms, tuners, tests, Android resources.
- `FtcRobotController/`: FTC application host, SDK samples and utility OpModes; indexed separately as vendor support.
- `settings.gradle`: includes both Android modules.
- `build.gradle`, `build.common.gradle`, `build.dependencies.gradle`: plugins, shared Android configuration, declared dependencies.
- `TeamCode/build.gradle`: team module, Sloth plugin and project-local Git settings during Gradle evaluation.
- `configs/`: robot hardware configuration XML; `libs/`: bundled support artifacts; `doc/`: upstream/legal material.
- `.github/workflows/` and `.githooks/`: commit rules and pull-request automation.

## Validation and limits

Inventory and symbols were generated from tracked files, with source hashes for freshness checks.
Comments are removed before declaration extraction, preserving source line numbers. Signatures and dependency edges are heuristic.
No compile, deployment, robot run or dependency resolution was performed for this documentation task.
Local settings, credentials, keystores, hidden tool state, build output and binary contents are excluded.
Atlas stores generated summaries and symbol outlines, not complete raw source files. Re-read current sources before editing.

## TeamCode file and symbol catalog

'''
    def file_section(j, relative=False):
        p = j['path']
        link = '../../' + p
        lines = [f"### {p[len(BASE):] if relative else p}", '', j['role'], '',
                 f"Source: [{p}]({link}); {j['lines']} lines; SHA-256 `{j['sha256']}`.", '',
                 'Types: ' + ', '.join(f"`{t['name']}` ({t['kind']}, line {t['line']})" for t in j['types']), '']
        for m in j['methods']:
            lines.append(f"- L{m['line']}: `{m['signature']}`")
        lines += ['', 'Internal dependency candidates: ' + (', '.join('`' + p + '`' for p in j['internal_dependencies']) or 'none detected'), '']
        return '\n'.join(lines)
    catalog = overview + '\n'.join(file_section(j, True) for j in team)
    (OUT / 'INDEX.md').write_text(catalog, encoding='utf-8')
    vendor = '# SDK / vendor source catalog\n\nGenerated outline; no SDK files modified.\n\n' + '\n'.join(file_section(j) for j in java if j not in team)
    (OUT / 'SDK-INDEX.md').write_text(vendor, encoding='utf-8')
    mm = ['# Project mind map', '', 'Generated from current tracked sources; open mindmap.html for a searchable expandable tree.', '', '```mermaid', 'mindmap', '  root((ElectroLarp Biobuzz))', '    TeamCode']
    for package, entries in sorted(package_groups.items()):
        mm.append('      ' + (package.replace('org.firstinspires.ftc.teamcode', '').lstrip('.') or 'Robot core'))
        for j in entries:
            mm.append('        ' + Path(j['path']).stem)
    mm += ['    SDK host', '      FtcRobotController', '      Samples and utility OpModes', '      Android resources', '    Build and dependencies']
    mm.extend('      ' + d.replace(':', ' ') for d in dependencies)
    mm += ['    Workflow and configuration', '      HardwareNames and configs', '      Git hooks and CI', '      Tuning guides', '```', '', '## Runtime relationships', '', '```mermaid', 'flowchart TD', '  OpModes[Alliance OpModes] --> Lifecycle[CommandOpMode / Ivy scheduler]', '  OpModes --> Robot', '  Robot --> Follower[PulseFollower]', '  Constants --> Follower', '  Follower --> Pinpoint[Pinpoint localizer]', '  Follower --> Drive[Mecanum / NoDrivetrain]', '  Follower --> Foresight', '  Robot --> Solver[ShooterCalc]', '  Solver --> Points[TunedShotTable / TunedShotPoints]', '  Solver --> Map[HiveAimTable / generated data]', '  Robot --> TurretCalc', '  Robot --> ShotCheck', '  Robot --> Mechanisms[Intake and transfer / Shooter / Hood / Turret / Latch / Flower]', '  Close --> Paths[ClosePaths / flower callback]', '  Tele --> Controls[GamepadEx / live trims / manual presets]', '  Robot --> Pose[OpModeStorage pose handoff]', '```', '']
    (OUT / 'MINDMAP.md').write_text('\n'.join(mm), encoding='utf-8')
    def branch(label, children):
        return '<details open><summary>' + html.escape(label) + '</summary><div class="children">' + ''.join(children) + '</div></details>'
    def leaf(j):
        methods = ''.join('<li><a href="../../' + html.escape(j['path']) + '#L' + str(m['line']) + '">L' + str(m['line']) + '</a> <code>' + html.escape(m['signature']) + '</code></li>' for m in j['methods'])
        return '<details><summary>' + html.escape(Path(j['path']).stem) + '</summary><div class="children"><p>' + html.escape(j['role']) + '</p><a href="../../' + html.escape(j['path']) + '">' + html.escape(j['path']) + '</a><ul>' + methods + '</ul><p>Dependency candidates: ' + html.escape(', '.join(Path(p).stem for p in j['internal_dependencies']) or 'none detected') + '</p></div></details>'
    tree = branch('TeamCode', [branch(pkg.replace('org.firstinspires.ftc.teamcode', '').lstrip('.') or 'Robot core', [leaf(j) for j in entries]) for pkg, entries in sorted(package_groups.items())])
    tree += branch('FTC SDK / vendor', [leaf(j) for j in java if j not in team])
    java_paths = {j['path'] for j in java}
    by_top = defaultdict(list)
    for p in paths:
        if p not in java_paths:
            by_top[p.split('/')[0] if '/' in p else 'Root build and metadata'].append('<p><a href="../../' + html.escape(p) + '">' + html.escape(p) + '</a></p>')
    tree += branch('Build, configuration, workflow and assets', [branch(k, v) for k, v in sorted(by_top.items())])
    page = '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>ElectroLarp-Biobuzz mind map</title><style>
body{font:15px/1.6 system-ui;margin:0;background:#101722;color:#e0e9f6}header,main{max-width:1200px;margin:auto;padding:24px}header{position:sticky;top:0;background:#101722f5;z-index:1;border-bottom:1px solid #334158}h1{margin:0;font-size:25px}a{color:#80caff}input,button{background:#1c293a;color:#fff;border:1px solid #52657f;border-radius:7px;padding:10px;margin:6px 6px 0 0}input{width:min(450px,85%)}details{margin:8px 0}summary{cursor:pointer;font-weight:650;padding:5px 10px;border-radius:5px;background:#1c293a}.children{margin-left:22px;border-left:2px solid #365574;padding-left:18px}code{overflow-wrap:anywhere}p{margin:8px 0}li{margin:5px 0}[hidden]{display:none!important}.meta{color:#a6b6cd;font-size:13px}</style><header><h1>ElectroLarp-Biobuzz · project mind map</h1><p class="meta">DATE · TEAM TeamCode Java files · SDK SDK Java files · FILES inventoried paths. Static source outline, not a resolved call graph.</p><input id="search" aria-label="Search files, methods and roles" placeholder="Search paths, mechanisms, methods…"><button id="expand">Expand all</button><button id="collapse">Collapse all</button><p><a href="INDEX.md">Team index</a> · <a href="SDK-INDEX.md">SDK index</a> · <a href="MINDMAP.md">Mermaid diagrams</a> · <a href="index.json">JSON index</a></p><div id="result" class="meta" role="status"></div></header><main id="tree">TREE</main><script>
const nodes=[...document.querySelectorAll('main details')];const search=document.getElementById('search');
function filter(){const q=search.value.trim().toLowerCase();let count=0;for(const n of [...nodes].reverse()){const own=n.querySelector(':scope > summary').textContent.toLowerCase();const matched=!q||n.textContent.toLowerCase().includes(q);n.hidden=!matched;if(q&&matched)n.open=true;if(matched&&own.includes(q))count++;}document.getElementById('result').textContent=q?count+' matching branches or files':'Expand a class to view methods, source links and dependency candidates.';}
search.addEventListener('input',filter);document.getElementById('expand').onclick=()=>nodes.forEach(n=>n.open=true);document.getElementById('collapse').onclick=()=>nodes.forEach(n=>n.open=false);filter();</script></html>'''
    page = page.replace('DATE', STAMP).replace('TEAM', str(len(team))).replace('SDK SDK', str(len(java) - len(team)) + ' SDK').replace('FILES', str(len(paths))).replace('TREE', tree)
    (OUT / 'mindmap.html').write_text(page, encoding='utf-8')
    atlas = [{'path': 'architecture/OVERVIEW.md', 'content': overview}]
    for pkg, entries in sorted(package_groups.items()):
        content = '# ' + pkg + '\n\nSnapshot ' + STAMP + '. Symbols are heuristic; verify current source before editing.\n\n' + '\n'.join(file_section(j) for j in entries)
        atlas.append({'path': 'packages/' + pkg + '.md', 'content': content})
    # Bound documents below Atlas's 100,000-character limit.
    for i in range(0, len(java), 15):
        group = java[i:i+15]
        atlas.append({'path': 'symbols/batch-' + str(i // 15 + 1).zfill(2) + '.md', 'content': '\n'.join(file_section(j) for j in group)})
    inv = '# Tracked non-sensitive repository inventory\n\n' + '\n'.join('- `' + p + '`' for p in paths)
    atlas.append({'path': 'inventory/FILES.md', 'content': inv})
    atlas.append({'path': 'architecture/MINDMAP.md', 'content': '\n'.join(mm)})
    for doc in atlas:
        if len(doc['content']) > 100000:
            raise ValueError('Atlas document exceeds limit: ' + doc['path'])
    (OUT / 'atlas-documents.json').write_text(json.dumps(atlas, indent=2), encoding='utf-8')
    print(json.dumps({'files': len(paths), 'team_java': len(team), 'sdk_java': len(java) - len(team), 'methods': sum(len(j['methods']) for j in java), 'edges': len(edges), 'atlas_documents': len(atlas), 'output': str(OUT)}))


if __name__ == '__main__':
    main()
