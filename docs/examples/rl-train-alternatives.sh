
# start direct-check
rwrl train hil_serl --config local/rl-direct.yaml --print-config
rwrl --station local/station.yaml --preset local/rl-direct.yaml doctor
# end direct-check

# start direct-actor
rwrl supervise --startup-s 60 -- \
  rwrl train hil_serl --config local/rl-direct.yaml --role actor \
  --sync-host "$LEARNER_HOST" --sync-port 6000
# end direct-actor

# start base-bc
BASE='checkpoint:local/bc-runs/first-bc/checkpoints/final.pt?algo=real_world_bc'
RUN=first-residual-bc
# end base-bc

# start base-act
BASE='act:local/bc-runs/first-act/checkpoints/final.pt?temporal_agg=1'
RUN=first-residual-act
# end base-act

# start residual-learner
rwrl train residual_hil_serl --config local/rl-task.yaml --role learner \
  --exp-name "$RUN" --run-id "$RUN" --sync-host 0.0.0.0 --sync-port 6000 \
  --base-provider "$BASE" --residual-variant hybrid --residual-action-scale 0.1 \
  --residual-bc-weight 0.1 --critic-only-updates 500
# end residual-learner

# start silri-learner
rwrl train silri --config local/rl-silri.yaml --role learner \
  --exp-name first-silri --run-id first-silri --sync-host 0.0.0.0 --sync-port 6000 \
  --demo-dataset-paths local/first-demo
# end silri-learner

# start residual-actor
rwrl train residual_hil_serl --config local/rl-task.yaml --role actor \
  --exp-name "$RUN" --run-id "$RUN" --sync-host "$LEARNER_HOST" --sync-port 6000 \
  --base-provider "$BASE" --residual-variant hybrid --residual-action-scale 0.1 \
  --residual-bc-weight 0.1 --critic-only-updates 500
# end residual-actor

# start silri-actor
rwrl train silri --config local/rl-silri.yaml --role actor \
  --exp-name first-silri --run-id first-silri --sync-host "$LEARNER_HOST" --sync-port 6000
# end silri-actor
