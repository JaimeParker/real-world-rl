# start help
rwrl train --help
export CUDA_VISIBLE_DEVICES=''
rwrl train hil_serl --help
rwrl daemon --help
# end help
# start daemon
# Robot PC: use the reviewed station and the watchdog budget from deployment.
rwrl --station local/station.yaml daemon --watchdog-s 5
# end daemon
# start learner
# Learner machine: same repository root, matching config and demo/sidecar.
export CUDA_VISIBLE_DEVICES=''
rwrl train hil_serl --config local/rl-task.yaml --role learner \
  --sync-host 0.0.0.0 --sync-port 6000 \
  --demo-dataset-paths local/first-demo.pkl
# end learner
# start actor
# Robot PC: replace with the learner's reachable LAN address; localhost if co-located.
LEARNER_HOST=REPLACE_WITH_LEARNER_LAN_IP
export CUDA_VISIBLE_DEVICES=''
rwrl train hil_serl --config local/rl-task.yaml --role actor \
  --sync-host "$LEARNER_HOST" --sync-port 6000
# end actor
# start inspect
python site/examples/rl-intervene-inspect.py local/rl-runs/first-hil/checkpoints/journal.sqlite3
# end inspect
# start smoke
PYTHONPATH=.:rl-garden CUDA_VISIBLE_DEVICES='' \
  ~/miniconda3/envs/rlg/bin/python site/examples/rl-train-smoke.py /tmp/rl-route-example
# end smoke
# start source-help
PYTHONPATH=.:rl-garden CUDA_VISIBLE_DEVICES='' \
  ~/miniconda3/envs/rlg/bin/python -m real_world_rl.cli train hil_serl --help
# end source-help
# start metrics
python site/examples/rl-train-metrics.py local/rl-runs
# end metrics
