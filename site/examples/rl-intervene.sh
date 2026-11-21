# start spacemouse
export CUDA_VISIBLE_DEVICES=''
rwrl train hil_serl --config local/rl-task.yaml --role actor \
  --sync-host "$LEARNER_HOST" --sync-port 6000 \
  --teleop-device spacemouse --spacemouse-mode tracker --teleop-event-device stdin
# end spacemouse
# start gello
export CUDA_VISIBLE_DEVICES=''
rwrl train hil_serl --config local/rl-task.yaml --role actor \
  --sync-host "$LEARNER_HOST" --sync-port 6000 \
  --teleop-device leader_cartesian --teleop-event-device stdin
# end gello
# start match
# On the learner; replace the whole ID with the correction record's record_id.
python site/examples/rl-intervene-inspect.py \
  local/rl-runs/first-hil/checkpoints/journal.sqlite3 --id 'REPLACE_WITH_RECORD_ID'
# end match
# start rehearsal
python site/examples/rl-intervene-inspect.py /tmp/rl-route-example/actor-audit.sqlite3 \
  --compare /tmp/rl-route-example/checkpoints/journal.sqlite3
# end rehearsal
