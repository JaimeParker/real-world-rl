
# start train
python -m rl_garden.models.act.train_trajectories \
  --demo-paths local/first-demo.pkl --total-iters 50000 --batch-size 256 \
  --num-queries 30 --backbone-weights none --save-freq 10000 \
  --log-dir local/bc-runs --exp-name first-act
# end train
