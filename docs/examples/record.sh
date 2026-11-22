# start record
# Selects the backend from station.teleop.backend (spacemouse or gello).
rwrl teleop run --record local/first-demo.pkl --num-successes 2 --max-episodes 5
# end record
# start events
# Before starting, create a new empty event file in a second terminal.
touch local/record-events.jsonl
rwrl teleop run --events-file local/record-events.jsonl --record local/events-demo.pkl --num-successes 1 --max-episodes 2
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
rwrl record --help
rwrl archive inspect --help
# end help

# start ordinary
rwrl supervise --log runs/record-ordinary/supervise.jsonl -- \
    rwrl record --config local/record-cartesian.yaml \
    --teleop-device leader_cartesian --teleop-event-device stdin \
    --episode-end-behavior auto --real_robot.watchdog_s 0.3 \
    --output-path local/ordinary-demo.pkl --num-successes 1
# end ordinary

# start ordinary-spacemouse
rwrl supervise --log runs/record-spacemouse/supervise.jsonl -- \
    rwrl record --config local/record-cartesian.yaml \
    --teleop-device spacemouse --teleop-options-yaml None \
    --teleop-event-device stdin --episode-end-behavior auto \
    --real_robot.watchdog_s 0.3 \
    --output-path local/ordinary-spacemouse.pkl --num-successes 1
# end ordinary-spacemouse
