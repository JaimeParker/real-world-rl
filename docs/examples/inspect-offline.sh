# start training-view
python - <<'PYCODE'
import json
from real_world_rl.dataset import Dataset, iter_transitions
dataset = Dataset('local/inspect-example/demo')
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
