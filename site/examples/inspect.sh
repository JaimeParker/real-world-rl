# start inspect
rwrl archive inspect local/first-demo.trajectory.sqlite3 --csv local/first-demo-steps.csv --measurements local/first-demo-measurements.csv
# end inspect
# start synthetic
# Requires the test extra; reuses the existing test's scripted trajectory.
mkdir -p local/inspect-example
python - <<'PYCODE'
import runpy
from pathlib import Path
fixture = runpy.run_path('tests/test_inspect_teleop_archive.py')
fixture['write_archive'](Path('local/inspect-example/demo.trajectory.sqlite3'),
                         contract=fixture['CARTESIAN_CONTRACT'])
PYCODE
rwrl archive inspect local/inspect-example/demo.trajectory.sqlite3 --csv local/inspect-example/steps.csv --measurements local/inspect-example/measurements.csv --drift-window-s 0.85 --stop-time-s 1002.4
# end synthetic
# start rows
python - <<'PYCODE'
import csv
import json
from pathlib import Path
path = Path('local/inspect-example/steps.csv')
with path.open(encoding='utf-8-sig', newline='') as stream:
    rows = list(csv.DictReader(stream))
human = [r for r in rows if '1' in r['human_action_mask']]
print('human steps:', human[0]['step_index'], '..', human[-1]['step_index'])
row = human[0]
print('mask:', row['human_action_mask'])
print('action:', row['action'])
# This fixture has state_fields=[joint_pos,tcp_pose], unlike the tutorial preset.
xyz = json.loads(row['state'])[0][7:10]
next_xyz = json.loads(row['next_state'])[0][7:10]
print('delta_xyz_mm:', [round((b-a)*1000, 3) for a,b in zip(xyz, next_xyz)])
PYCODE
# end rows
# start pkl
# Only open a pickle you recorded yourself or otherwise trust.
python - <<'PYCODE'
import json
import pickle
from pathlib import Path
path = Path('local/first-demo.pkl')
with path.open('rb') as stream:
    rows = pickle.load(stream)
contract = json.loads(path.with_suffix('.contract.json').read_text())
print('transitions:', len(rows))
print('keys:', sorted(rows[0]) if rows else [])
print('first human_action_mask:', rows[0].get('human_action_mask') if rows else None)
print(json.dumps(contract, indent=2, ensure_ascii=False))
PYCODE
# end pkl
