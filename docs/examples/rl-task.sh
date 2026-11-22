# start prepare
# Replace <uuid> with the directory reported by record.rst.
python docs/examples/rl-task-prepare.py \
  'local/first-demo.session-<uuid>/resolved-config.yaml' local/rl-task.yaml \
  --remote 127.0.0.1:50051
# end prepare
# start preflight
export CUDA_VISIBLE_DEVICES=''
rwrl train hil_serl --config local/rl-task.yaml --role learner \
  --demo-dataset-paths local/first-demo.pkl --print-config
# end preflight
# start labels
python docs/examples/rl-intervene-inspect.py local/first-demo.trajectory.sqlite3
# end labels
# start files
ls -d local/first-demo.session-*/
ls -l local/first-demo.pkl local/first-demo.contract.json
# end files
