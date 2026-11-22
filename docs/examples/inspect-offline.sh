# start record
# Offline only: uses the recorder's existing test double, not robot hardware.
python - <<'PY'
import runpy
import sys
from pathlib import Path
import pytest
sys.path.insert(0, 'tests')
fixture = runpy.run_path('tests/test_real_demo_recorder.py')
with pytest.MonkeyPatch.context() as patch:
    fixture['test_run_recorder_builds_the_actor_env_saves_and_closes'](
        Path('local/inspect-example'), patch)
PY
# end record
# start pkl
python - <<'PY'
import json
import pickle
from pathlib import Path
path = Path('local/inspect-example/demos.pkl')
with path.open('rb') as stream:
    rows = pickle.load(stream)
contract = json.loads(path.with_suffix('.contract.json').read_text())
print('transitions:', len(rows))
print('keys:', sorted(rows[0]))
print('first human_action_mask:', rows[0]['human_action_mask'])
print(json.dumps(contract, indent=2, ensure_ascii=False))
PY
# end pkl
