# start prepare
# Replace <uuid> with the directory printed in bc-data; do not type it literally.
python site/examples/bc-train-prepare.py 'local/first-demo.session-<uuid>/resolved-config.yaml' local/first-demo.pkl local/bc-train.yaml
# end prepare
# start synthetic-prepare
python site/examples/bc-train-prepare.py local/bc-synthetic/resolved-config.yaml local/bc-synthetic/demos.pkl local/bc-train.yaml
# end synthetic-prepare
# start train
export CUDA_VISIBLE_DEVICES=""
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
rwrl train real_world_bc --config local/bc-train.yaml
# end train
# start help
rwrl train real_world_bc --help
# end help
# start logs
python site/examples/bc-train-logs.py local/bc-runs/pick_and_place/first-bc
ls local/bc-runs/first-bc
ls local/bc-runs/first-bc/checkpoints
# end logs
# start synthetic-train
# The source-only rlg environment needs explicit backend registration before preflight.
export PYTHONPATH=.:rl-garden
export CUDA_VISIBLE_DEVICES=""
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
python -c 'import real_world_rl.envs.backends.real_robot; from real_world_rl.training.__main__ import main; main()' real_world_bc --config local/bc-train.yaml
# end synthetic-train
