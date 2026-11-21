# start check
export CUDA_VISIBLE_DEVICES=""
export OMP_NUM_THREADS=1
python site/examples/bc-check.py local/bc-runs/first-bc/checkpoints/final.pt local/first-demo.pkl --row 0 --record local/bc-runs/first-bc/offline-check.json
# end check
# start synthetic
python site/examples/bc-check.py local/bc-runs/first-bc/checkpoints/final.pt local/bc-synthetic/demos.pkl --row 0 --record local/bc-runs/first-bc/offline-check.json
# end synthetic
# start record
python -m json.tool local/bc-runs/first-bc/offline-check.json
# end record
