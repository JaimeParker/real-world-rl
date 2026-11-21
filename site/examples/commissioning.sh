# Material for the upcoming device chapters; run only the relevant block.
# start hid
python -c 'import hid; print(hid.enumerate())'
ls -l /dev/hidraw*
id
# end hid
# start serial
ls -l /dev/serial/by-id/
id
# end serial
# start dialout
sudo usermod -aG dialout "$USER"
# end dialout
# start pico
python -c 'import zmq; print(zmq.zmq_version())'
# Replace this address with the publisher address shown by the Pico-side application.
python examples/run_twist_input_inspector.py --zmq-url tcp://REPLACE_WITH_PICO_HOST:7777
# end pico
