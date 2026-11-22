
# start residual
rwrl supervise --startup-s 60 -- \
  rwrl deploy --checkpoint local/rl-runs/first-residual-act/checkpoints/bundle_manifest.json \
  --algo residual_hil_serl_hybrid \
  --base-provider 'act:local/bc-runs/first-act/checkpoints/final.pt?temporal_agg=1' \
  --no-enable-teleop --episodes 1 --max-steps-per-episode 10
# end residual

# start assisted
rwrl supervise --startup-s 60 -- \
  rwrl deploy --checkpoint local/rl-runs/first-hil/checkpoints/bundle_manifest.json \
  --algo rlpd_hybrid --teleop-device leader_cartesian --teleop-event-device stdin \
  --episodes 1 --max-steps-per-episode 10
# end assisted

# start act
rwrl supervise --startup-s 60 -- \
  rwrl deploy-act --config local/rl-task.yaml \
  --checkpoint local/bc-runs/first-act/checkpoints/final.pt \
  --demo-contract local/first-demo.contract.json --no-enable-teleop \
  --episodes 1 --max-steps-per-episode 10
# end act
