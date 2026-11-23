# start local-data
ls -l local/rl-runs/first-hil/actor/outbox.sqlite3*
ls -l local/rl-runs/first-hil/checkpoints/journal.sqlite3*
cat local/rl-runs/first-hil/checkpoints/bundle_manifest.json
python docs/examples/rl-intervene-inspect.py local/rl-runs/first-hil/checkpoints/journal.sqlite3
# end local-data
# start resume
export CUDA_VISIBLE_DEVICES=''
rwrl train hil_serl --config local/rl-task.yaml --role learner \
  --sync-host 0.0.0.0 --sync-port 6000 \
  --checkpoint-dir local/rl-runs/first-hil/checkpoints \
  --load-checkpoint local/rl-runs/first-hil/checkpoints/bundle_manifest.json \
  --demo-dataset-paths local/first-demo
# end resume
# start eval
# Robot PC; stop its actor first. Copy the COMPLETE saved run here if needed.
export CUDA_VISIBLE_DEVICES=''
rwrl train hil_serl --config local/rl-task.yaml --role eval \
  --load-checkpoint local/rl-runs/first-hil/checkpoints/bundle_manifest.json \
  --eval-n-trajs 5 --seed 1
# end eval

# start residual-eval
rwrl train residual_hil_serl --config local/rl-task.yaml --role eval \
  --exp-name "$RUN" --run-id "$RUN" --base-provider "$BASE" \
  --residual-variant hybrid --residual-action-scale 0.1 \
  --residual-bc-weight 0.1 --critic-only-updates 500 \
  --load-checkpoint "local/rl-runs/$RUN/checkpoints/bundle_manifest.json" --eval-n-trajs 5
# end residual-eval

# start silri-eval
rwrl train silri --config local/rl-silri.yaml --role eval \
  --exp-name first-silri --run-id first-silri \
  --load-checkpoint local/rl-runs/first-silri/checkpoints/bundle_manifest.json --eval-n-trajs 5
# end silri-eval
