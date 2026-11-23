# start prepare
# Replace <uuid> with the directory reported by record.rst.
python docs/examples/rl-task-prepare.py \
  'local/first-demo.session-<uuid>/resolved-config.yaml' local/rl-task.yaml \
  --remote 127.0.0.1:50051
# end prepare
# start preflight
export CUDA_VISIBLE_DEVICES=''
rwrl train hil_serl --config local/rl-task.yaml --role learner \
  --demo-dataset-paths local/first-demo --print-config
# end preflight
# start labels
python - <<'PYCODE'
import json
from pathlib import Path
for path in sorted(Path('local/first-demo').glob('episode_*/meta.json')):
    meta = json.loads(path.read_text())
    outcome = meta.get('extra', {}).get('outcome', {})
    print(json.dumps({'episode': meta['index'], 'status': meta['status'],
                      'discard_reason': meta['discard_reason'], 'success': meta['success'],
                      'task_success': outcome.get('task_success'),
                      'outcome_source': outcome.get('outcome_source'),
                      'end_reason': outcome.get('end_reason')}, sort_keys=True))
PYCODE
# end labels
# start files
ls -d local/first-demo.session-*/
ls local/first-demo/dataset.json
rwrl dataset list local/first-demo
# end files
