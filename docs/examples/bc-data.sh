# start inspect
rwrl archive inspect local/first-demo
python docs/examples/bc-data.py local/first-demo
# end inspect
# start session
ls local/first-demo.session-*/resolved-config.yaml
cat local/task.yaml
cat local/station.yaml
# end session
# start synthetic
python docs/examples/bc-data-synthetic.py local/bc-synthetic
python docs/examples/bc-data.py local/bc-synthetic/demos
# end synthetic
# start gello
# Only when you recorded the joint-space extension in the GELLO chapter.
python docs/examples/bc-data.py local/gello-joint
# end gello
# start source-env
export PYTHONPATH=.:rl-garden
export CUDA_VISIBLE_DEVICES=""
# end source-env
