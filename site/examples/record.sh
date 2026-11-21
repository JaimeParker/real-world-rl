# start record
# Selects the backend from station.teleop.backend (spacemouse or gello).
rwrl --station local/station.yaml --preset local/task.yaml teleop run --record local/first-demo.pkl --num-successes 2 --max-episodes 5
# end record
# start events
# Before starting, create a new empty event file in a second terminal.
touch local/record-events.jsonl
rwrl --station local/station.yaml --preset local/task.yaml teleop run --events-file local/record-events.jsonl --record local/events-demo.pkl --num-successes 1 --max-episodes 2
# end events
# start success
# In the second terminal, after completing the task:
printf '%s\n' '{"event":"success"}' >> local/record-events.jsonl
# end success
# start next
# Only after seeing EPISODE_HOLD and preparing the reset area:
printf '%s\n' '{"event":"next"}' >> local/record-events.jsonl
# end next
# start quit
printf '%s\n' '{"event":"quit"}' >> local/record-events.jsonl
# end quit
# start help
python -m real_world_rl.demos --help
rwrl archive inspect --help
# end help
