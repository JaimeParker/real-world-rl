# start list
rwrl dataset list local/first-demo
# end list
# start view
rwrl dataset view local/first-demo
# end view
# start view-options
rwrl dataset view local/first-demo -e 0 -e 2
rwrl dataset view local/first-demo --web
rwrl dataset view local/first-demo --save local/first-demo.rrd
# end view-options
# start forward
# On your own computer: forward both the web viewer and the gRPC port, then open the printed URL.
ssh -L 9090:localhost:9090 -L 9876:localhost:9876 USER@ROBOT_PC
# end forward
# start inspect
rwrl archive inspect local/first-demo --csv local/first-demo-steps.csv --measurements local/first-demo-measurements.csv
# end inspect
# start synthetic
# Requires the test extra; reuses the existing test's scripted trajectory, written as a dataset.
mkdir -p local/inspect-example
python - <<'PYCODE'
import runpy
from pathlib import Path
fixture = runpy.run_path('tests/test_inspect_teleop_archive.py')
fixture['write_dataset'](Path('local/inspect-example/demo'), named=True)
PYCODE
rwrl archive inspect local/inspect-example/demo --csv local/inspect-example/steps.csv --measurements local/inspect-example/measurements.csv --drift-window-s 0.85 --stop-time-s 1002.4
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
# start training-view
python - <<'PYCODE'
import json
from real_world_rl.dataset import Dataset, iter_transitions
dataset = Dataset('local/first-demo')
rows = list(iter_transitions(dataset, frame_stack=1))
print('episodes:', [(info.index, info.meta['status']) for info in dataset.episodes()])
print('transitions:', len(rows))
print('keys:', sorted(rows[0]) if rows else [])
print('first human_action_mask:', rows[0]['human_action_mask'] if rows else None)
print('segment ends:', [row['step_index'] for row in rows if row.get('training_segment_end')])
print(json.dumps(dataset.header['action_contract'], indent=2, ensure_ascii=False))
print('obs_spec:', json.dumps(dataset.header['obs_spec']))
PYCODE
# end training-view
