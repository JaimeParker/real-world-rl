
# start export
# The ACT trainer reads a pkl plus its contract sidecar, so export one from the dataset first.
rwrl dataset export local/first-demo --format pkl --out local/first-demo.pkl
# end export
# start train
python -m rl_garden.models.act.train_trajectories \
  --demo-paths local/first-demo.pkl --total-iters 50000 --batch-size 256 \
  --num-queries 30 --backbone-weights none --save-freq 10000 \
  --log-dir local/bc-runs --exp-name first-act
# end train
