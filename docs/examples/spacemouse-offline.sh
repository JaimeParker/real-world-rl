# start read
rwrl --mock --station docs/examples/spacemouse-mock.yaml teleop read --duration-s 0.2 --rate-hz 10
# end read
# start calibrate
mkdir -p local/spacemouse-offline
python - <<'PY'
import json
import runpy
from pathlib import Path
fixture = runpy.run_path('tests/test_spacemouse_calibration.py')
Path('local/spacemouse-offline/samples.json').write_text(json.dumps(fixture['samples'](0.37)))
PY
rwrl --station docs/examples/spacemouse-mock.yaml teleop calibrate --backend spacemouse --profile synthetic --frame base --samples local/spacemouse-offline/samples.json
# end calibrate
