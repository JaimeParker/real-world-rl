# start local-data
ls -l local/rl-runs/first-hil/actor/outbox.sqlite3*
ls -l local/rl-runs/first-hil/checkpoints/journal.sqlite3*
cat local/rl-runs/first-hil/checkpoints/bundle_manifest.json
python site/examples/rl-intervene-inspect.py local/rl-runs/first-hil/checkpoints/journal.sqlite3
# end local-data
# start resume
export CUDA_VISIBLE_DEVICES=''
rwrl train hil_serl --config local/rl-task.yaml --role learner \
  --sync-host 0.0.0.0 --sync-port 6000 \
  --checkpoint-dir local/rl-runs/first-hil/checkpoints \
  --load-checkpoint local/rl-runs/first-hil/checkpoints/bundle_manifest.json \
  --demo-dataset-paths local/first-demo.pkl
# end resume
# start eval
# Robot PC; stop its actor first. Copy the COMPLETE saved run here if needed.
export CUDA_VISIBLE_DEVICES=''
rwrl train hil_serl --config local/rl-task.yaml --role eval \
  --load-checkpoint local/rl-runs/first-hil/checkpoints/bundle_manifest.json \
  --eval-n-trajs 5 --seed 1
# end eval
