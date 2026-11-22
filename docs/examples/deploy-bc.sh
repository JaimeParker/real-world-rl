# start environment
export PYTHONPATH=.:rl-garden
export CUDA_VISIBLE_DEVICES=""
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
# end environment
# start inspect
python docs/examples/deploy-bc.py local/bc-runs/first-bc/checkpoints/final.pt
python -m json.tool local/bc-runs/first-bc/offline-check.json
cat local/task.yaml
cat local/station.yaml
# end inspect
# start mismatch
python docs/examples/deploy-bc.py local/bc-runs/first-bc/checkpoints/final.pt --xyz-scale-m 0.04
# end mismatch
# start offline
python docs/examples/bc-check.py local/bc-runs/first-bc/checkpoints/final.pt local/first-demo.pkl --row 0 --record local/bc-runs/first-bc/deploy-bc-offline.json
# end offline
# start synthetic
python docs/examples/bc-check.py local/bc-runs/first-bc/checkpoints/final.pt local/bc-synthetic/demos.pkl --row 0 --record local/bc-runs/first-bc/deploy-bc-offline.json
# end synthetic
# start help
rwrl deploy --help
# end help
# start cli
rwrl deploy --checkpoint local/bc-runs/first-bc/checkpoints/final.pt \
  --algo real_world_bc --device cpu --real-robot.remote 127.0.0.1:50061 \
  --episodes 1 --max-steps-per-episode 10 --no-enable-teleop
# end cli
# start episode
set -o pipefail
mkdir -p local
python docs/examples/deploy-bc.py local/bc-runs/first-bc/checkpoints/final.pt \
  --remote 127.0.0.1:50061 --run --output local/deploy-bc-episode \
  2>&1 | tee local/deploy-bc-episode.log
# end episode
# start records
cat local/deploy-bc-episode/steps.jsonl
python -m json.tool local/deploy-bc-episode/result.json
rg '\[deploy\]' local/deploy-bc-episode.log
# end records
