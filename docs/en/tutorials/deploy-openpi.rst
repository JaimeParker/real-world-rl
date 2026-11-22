Deploy an existing openpi / RLinf policy
================================================================================

You have a π0.5 checkpoint but are not sure whether it can drive the Franka in front of you.
First match the weights to the training task, then have the GPU service return an action chunk, and only then connect to the station for execution.
This chapter continues with ``local/station.yaml`` from :doc:`station`;
keep ``local/task.yaml`` to check the existing observation and action contract. This entry point does not read it automatically.
For a BC checkpoint trained in this repository, use :doc:`deploy`.

.. tip::

   Device commands in this chapter omit ``--station``: when run from the repository root, they use the station remembered by ``rwrl use``
   in Step 10 of :doc:`station`; without a remembered station, they also find ``local/station.yaml`` directly. To use another file, pass ``--station`` explicitly.

1. Identify the checkpoint you received
--------------------------------------------------------------------------------

Record four things first so you do not confuse “can read the files” with “can execute on this robot.”
A ``checkpoint`` is the saved model weights and accompanying assets; a ``loader`` is the code that restores those files into a model.
On the machine storing the weights, replace the path placeholder with the actual directory, then list files and text configurations:

.. tutorial-test: skip
.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start inspect
   :end-before: # end inspect

.. list-table:: Four checks
   :header-rows: 1
   :widths: 22 78

   * - What to record
     - How to check it
   * - Weight layout
     - Save the directory listing, model source, and version. Distinguish model parameters, training recovery state, configuration, and assets; verify exact filenames, shards, and formats. Do not judge loadability from extensions alone.
   * - Loader
     - Find the loader specified by that version's training/export instructions and confirm that the server configuration registry contains the target configuration. Record the source commit and startup command.
   * - Normalization statistics
     - Find the asset directory and statistics identifier referenced by the training configuration. Check state/action field order, dimensions, and units. Record the dataset the statistics belong to; verify filenames, selection rules, and behavior when missing.
   * - Training task
     - Record the objects, goal, initial layout, language instruction, camera views, and state/action contract. Even with the same Franka model, match every item.

**What you should see**: a directory listing, loader, statistics, and task description traceable to the same training configuration.
This chapter does not supply fabricated weight-directory output; check the version you actually received.

**If it doesn't**: no text match does not mean no statistics exist; continue inspecting the configuration and assets shipped with the model.
If you have only training recovery files, find that version's export workflow first. If you have only a model name, ask the weight provider for the training configuration.
If any of the four items is missing, stay at the offline-check stage.

2. From official pi05_base to RLinf, then check Pick_Red
--------------------------------------------------------------------------------

Use official ``pi05_base`` as the conversion starting point. Check “changing the weight representation” separately from “adapting to the on-site task.”
This repository does not contain its weights, conversion scripts, or upstream loader. The following is the delivery chain you must verify during conversion,
not a conversion command that has already been tested successfully.

.. literalinclude:: ../../examples/en/deploy-openpi-output.txt
   :language: text
   :start-after: # start conversion
   :end-before: # end conversion

Complete these steps in order:

#. Inspect the ``pi05_base`` directory as in the previous section and confirm that the source loader can restore the model.
   **Needs verification**: this version's original weight layout, download source, and exact loader.
#. In your RLinf checkout, find the conversion entry point for that source format. Check inputs, outputs, and dependency versions.
   **Needs verification**: conversion script path, command arguments, parameter mapping, and target format. This repository has no conversion command you can reuse directly.
#. Reload the converted artifact with the target loader, compare outputs for the same fixed input, and check whether normalization assets need to be copied separately.
   **Needs verification**: numerical tolerance, statistics-asset handling, and model transformations. Successful conversion alone does not prove the model learned Pick_Red.
#. To deploy through an openpi WebSocket service, also check whether that checkout's
   ``serve_policy.py`` can load the RLinf artifact or requires an additional export. **Needs verification**: export/server-loading compatibility at this step.

The service example supported by this repository's :doc:`../reference/deploy-openpi` and its Chinese version is
``pi05_franka_pnp``. It requires the server to register ``LeRobotRealworldDataConfig``,
``RealworldInputs``, and ``RealworldOutputs``. This repository's ``profiles.py`` explicitly maps
``rlinf_franka`` to this input family; a configuration using ``FrankaEEInputs`` has a different contract.
This does not prove that any model named π0.5 suits this profile.
The client in this repository only assembles observations and exchanges protocol messages; it does not find normalization statistics files for you.
Check in the server configuration how statistics are applied to inputs and outputs.

Now place the single-arm Franka task configuration for ``RLinf-Pi05-Pick_Red`` alongside it and check each item:

* Which configuration, loader, and statistics assets the model actually selects; whether it really uses the ``RealworldInputs`` contract above.
* Whether it uses single-arm seven-dimensional Cartesian actions; the first six dimensions' frame, incremental rotation, and scales; the final dimension's open/close thresholds.
* Whether it needs two images and where each comes from; whether crop, color order, size, and language instruction match.
* Whether the reset reference pose, gripper travel, control frequency, object positions, and target positions match the site.

**Needs verification**: the outline records that an upstream Pick_Red hardware configuration was found, but this repository contains neither its configuration text nor a pinned version.
That does not establish that it is ``pi05_franka_pnp``, nor that the example prompt below is its training instruction.
Save the configuration values and versions for every item above.

**Independent verification item: loading RLinf-Pi05-Pick_Red through WebSocket and completing inference with this repository has not been verified.**
To close this item, start the service with the actual weights, matching loader, and statistics, complete this chapter's dry-run,
and then check the input/output contract with on-site observations. Fake policies in tests do not satisfy this verification.

**What you should see**: an explicit loader for each of the source weights, converted artifact, and service-loadable artifact;
a Pick_Red configuration that maps item by item to the contract in Section 7.

**If it doesn't**: if converted weights cannot load, check source/target versions and parameter mapping first. If statistics are missing, supply the training assets.
If the inputs use ``FrankaEEInputs``, stop applying ``rlinf_franka``; a separate adapter must be implemented and verified.
This chapter uses only this repository's documentation and code; it does not infer executable commands from upstream details not included here.
For deeper reference, see :doc:`../reference/deploy-openpi`.

3. Start the policy service on the GPU
--------------------------------------------------------------------------------

Let the GPU handle inference and keep the NUC beside the robot to handle devices and limits. Distinguish the two connections first.
The ``daemon`` is the NUC's device service; a ``profile`` is this repository's rule for turning device observations into policy inputs.

.. literalinclude:: ../../examples/en/deploy-openpi-output.txt
   :language: text
   :start-after: # start topology
   :end-before: # end topology

The GPU needs two environments: an openpi environment to load weights and an rwrl environment to run the deployment loop.
In the GPU's rwrl environment, install client dependencies from the repository root and inspect the help used in this chapter:

.. tutorial-test: skip
.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start install
   :end-before: # end install

.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start help
   :end-before: # end help

``jpeg`` supplies image decoding and resizing dependencies; keep the NUC device dependencies from :doc:`install`.
Below is an excerpt of deploy help actually run locally. Compare the complete
:download:`deploy help <../../examples/deploy-openpi-help.txt>`,
:download:`daemon help <../../examples/deploy-openpi-daemon-help.txt>`, and
:download:`doctor help <../../examples/deploy-openpi-doctor-help.txt>`:

.. literalinclude:: ../../examples/deploy-openpi-help.txt
   :language: text
   :lines: 10-19

In the GPU's openpi checkout and environment, start the service with the verified export directory:

.. tutorial-test: skip
.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start serve
   :end-before: # end serve

This command comes from this repository's deployment documentation. **Needs verification**: the current upstream checkout's command line and startup logs;
this repository has no ``scripts/serve_policy.py``, so its help and actual weight loading could not be run here.
The documentation says the official service listens on all interfaces without authentication by default; verify this behavior for the upstream version too.
On site, restrict port 8000 access to the clients that need it.

**What you should see**: the service finishes loading and keeps running; ``link.policy`` in Section 6 can read its metadata.
Startup logs vary by upstream version; no success output is invented here.

**If it doesn't**: if the configuration name is missing, use a checkout with RLinf configuration registration.
If weights or statistics fail to load, return to Sections 1 and 2. If ports differ, change both the server port and
the client's ``--policy-port``. ``--api-key`` only sends an authentication header to a gateway/service that supports it; it cannot enable server-side authentication.

4. Start the daemon on the NUC
--------------------------------------------------------------------------------

Write the bind address into the existing station so device checks, the service, and SSH forwarding use the same port.
Add the block below at the top level of the ``local/station.yaml`` filled in and verified in :doc:`station`,
alongside ``type`` and ``cameras``. Do not overwrite your measured home and workspace with this chapter's template.

.. literalinclude:: ../../examples/en/deploy-openpi-station.yaml
   :language: yaml
   :start-after: # start daemon
   :end-before: # end daemon

.. list-table::
   :header-rows: 1

   * - Fields you must change
     - How to obtain the value
   * - ``daemon.bind``
     - This chapter uses the NUC loopback address ``127.0.0.1:50051``. If the port is occupied, identify its owner with ``link.daemon.listen``, choose a free port, and update the tunnel target and doctor's ``--remote-bind`` together.

Leave the other fields unset: ``daemon.token`` defaults to no token. If a token is needed, set it in the station
and pass the same value to doctor's ``--token`` and deployment's ``--real-robot.token``.
``--watchdog-s`` is a daemon command-line argument, not a field in this YAML block; the CLI default is 0.5 seconds.
The 2 seconds below comes from the deployment documentation example and must be reviewed against the measured intervals in Section 9.
Without ``daemon.bind``, the default is ``0.0.0.0:50051``, so the loopback address is explicit here.

Run this in the NUC's activated device environment:

.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start daemon
   :end-before: # end daemon

**What you should see**: device preflight checks, then the service line below. This is an example using the source-code format:

.. literalinclude:: ../../examples/en/deploy-openpi-output.txt
   :language: text
   :start-after: # start daemon
   :end-before: # end daemon

**If it doesn't**: on ``preflight failed``, follow :doc:`station` to fix the handshake or camera feedback.
If ``link.daemon.listen`` finds no listener, check whether the process exited, the configuration path, and bind.
If devices are occupied, stop the original control program before starting the service.

5. Open an SSH tunnel from the GPU
--------------------------------------------------------------------------------

Forward the GPU's local 50061 to the NUC's local 50051 so the deployment loop can use gRPC through the tunnel.
First confirm the site's NUC SSH user, hostname, host fingerprint, and login key, then run this in a separate GPU terminal:

.. tutorial-test: skip
.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start tunnel
   :end-before: # end tunnel

**What you should see**: the command stays running, usually without output; doctor's ``link.daemon`` should pass next.
The two ``127.0.0.1`` addresses belong to the GPU and NUC respectively; policy port 8000 does not use this forward.

**If it doesn't**: if the local port is occupied, inspect existing tunnels. If authentication fails, check the SSH user and key.
If the host key changes, verify the NUC fingerprint first. If ``link.forward`` reports forwarding prohibited, check
``link.sshd`` on the NUC, including ``AllowTcpForwarding``, ``DisableForwarding``, ``PermitOpen``,
and user-specific rules.

6. Read doctor on both sides, then run a dry-run
--------------------------------------------------------------------------------

First confirm the device link and policy handshake, then send one synthetic observation to isolate network issues from model-input issues.
Run this on the NUC:

.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start doctor-nuc
   :end-before: # end doctor-nuc

Run this in the GPU's rwrl environment:

.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start doctor-gpu
   :end-before: # end doctor-gpu

``--link`` is the daemon address as seen by the current machine. The GPU's ``--ssh`` really logs into the NUC and tests a temporary forward.
This chapter's verification did not perform a remote login; it captured failure lines only with the following local empty-port command:

.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start doctor-empty
   :end-before: # end doctor-empty

.. literalinclude:: ../../examples/deploy-openpi-doctor-gpu.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

Below is an excerpt of actual output from the same development machine simulating the NUC role: it uses a temporary fake station,
declares a loopback ``daemon.bind``, and runs the NUC form of the ``--link`` command without starting a service or using
``--mock``. ``--mock`` skips link checks and cannot capture these lines.

.. literalinclude:: ../../examples/deploy-openpi-doctor-nuc.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

.. list-table:: Read results by row
   :header-rows: 1
   :widths: 30 70

   * - Row name
     - What a pass means; what to do on failure
   * - ``link.daemon.bind`` / ``link.daemon.listen``
     - NUC configuration and listening process. If no listener exists, fix the daemon first; a bind pass does not mean the service has started.
   * - ``link.sshd`` / ``link.firewall``
     - NUC SSH forwarding permissions and inbound rules. Skipping firewall for a loopback address is expected; there is no need to expose 50051 to the LAN.
   * - ``link.ssh`` / ``link.tunnel`` / ``link.forward``
     - GPU login, the current tunnel, and an independent temporary forward. If temporary forwarding passes but the current link fails, check the local forwarding port and service state.
   * - ``link.daemon`` / ``link.deps``
     - The device description was retrieved; communication dependencies can be imported. For a token error, align tokens on both sides using the ``UNAUTHENTICATED (token mismatch)`` hint.
   * - ``link.preprocess``
     - Whether the NUC can use OpenCV preprocessing. If unavailable, it may send full-size images for the GPU to process; remeasure bandwidth.
   * - ``link.quality``
     - RTT from 10 describe calls and 3 observation transfers; post-reset gap is three times p95 and does not yet include policy inference. Fix preceding FAIL rows before reading timing.
   * - ``link.policy``
     - WebSocket handshake and metadata only; no inference request. On failure, check the server address, proxy, and service logs.
   * - ``link.proxy``
     - Proxy bypass and SSH fake-IP risks; see Section 9 for handling them.

The NUC's direct loopback check has no SSH configuration, so ``link.ssh`` and ``link.forward`` are skipped;
a ``link.tunnel`` hint when no tunnel service exists does not mean the NUC must create another tunnel.
Focus on server listening rows and ``link.daemon``; perform tunnel checks on the GPU.
See the link-check description in :doc:`../reference/doctor` for complete row definitions.
``link.quality`` may connect idle devices to sample observations; it disconnects any connection it established when the check finishes.

Then run one synthetic-input inference on the GPU:

.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start dry-run
   :end-before: # end dry-run

**What you should see**: ``dry-run actions=(chunk length, 7)``, plus client round-trip and server timings.
Below is actual output from the fake policy and protocol fixture in ``tests/test_deploy_openpi.py``;
the chunk length 3 and server time 3.0 ms both come from the fake. It did not load π0.5:

.. literalinclude:: ../../examples/deploy-openpi-dry-run.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

dry-run uses zero state and black images. It needs only the policy service and does not create a robot environment.
Actions must be a finite two-dimensional array ``(horizon, action_dim)``; passing does not show that scales and task match.
``--policy-timeout-s`` defaults to 30 seconds for policy connection and receive; it differs from the robot-reconnection
``--reconnect-timeout-s``. A policy timeout exits rather than entering the robot WAITING flow.

**If it doesn't**: for ``could not connect to openpi``, use ``link.policy`` to fix the address and proxy.
For ``openpi server error during inference``, check server input keys, statistics, and model logs.
For ``expected finite actions shaped (horizon, 7)``, check the output layout; do not slice dimensions just to make the count fit.

7. Align station, profile, and action mapping
--------------------------------------------------------------------------------

Before motion, map every policy input and output to real devices. In particular, do not reuse the previous chapter's single camera and action scales without checking.
On the NUC, query the daemon already running:

.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start mapping
   :end-before: # end mapping

**What you should see**: actual camera names, home, workspace, and per-step limits.
The ``wrist`` in :doc:`station` is only one camera; this chapter's service example defaults to
``wrist_1`` and ``wrist_2``. If you have only one, check whether the model allows the second view to be absent;
do not duplicate the same frame as two cameras and claim a match.
For a new camera, follow the station chapter to discover its serial, verify its image, and then fill in the station.
In ``--camera-keys``, put actual daemon camera names on the left and policy keys from the training configuration on the right.
For example, if ``status`` lists ``wrist`` and ``side``, change the left-hand names to those two.

.. list-table:: This repository's ``rlinf_franka`` contract
   :header-rows: 1
   :widths: 26 74

   * - Input/output
     - Order, units, and processing
   * - ``observation/state``: 19 dimensions
     - Gripper width 1 (m), TCP force 3, TCP pose 6 (XYZ relative to reset and xyz Euler angles in rad), TCP torque 3, TCP velocity 6 (inverse adjoint transform into the end-effector frame). Force and torque rotate from the base into the stiffness frame.
   * - Two images
     - Defaults: ``wrist_1 → observation/image``, ``wrist_2 → observation/extra_view_image``. Center square crop, OpenCV linear resize to 128 × 128, HWC uint8 BGR; use ``--crop-regions`` for proportional crop bounds.
   * - ``prompt``
     - Filled from the command-line language instruction; check it against the training task. The documented upstream model-side resize to 224 × 224 needs verification.
   * - Actions 0–5
     - The first six dimensions of the seven-dimensional action undergo the current-pose adjoint transform, clipping to [-1, 1], and translation/rotation scaling, then the Euler increment left-multiplies the measured orientation; this differs from the station chapter's ``current`` semantics.
   * - Action 6
     - After multiplication by ``gripper_action_scale``, reaching +0.5 opens and reaching −0.5 closes; the middle range holds. Measured open/closed state is also checked. Adjust the threshold with ``--real-robot.rlinf-gripper-threshold``; the Hand uses binary mode.

``--stiffness-quat`` defaults to the identity quaternion, assuming the stiffness frame aligns with the TCP axes. If you have a calibrated rotation on site,
supply its xyzw quaternion relative to the TCP. Keep the training crop, colors, and camera order.
Default translation/rotation scales are 0.02 m/0.1 rad; the deployment documentation's RLinf SFT example explicitly uses 1.0/1.0.
**These are examples, not confirmed Pick_Red parameters.** Use the verified training configuration,
while retaining the station workspace, per-step limits, and safe home.

``generic`` instead concatenates native-unit state according to ``--real-robot.state-fields`` and defaults to 224 × 224,
Pillow bilinear resizing, and RGB; actions follow the environment configuration. Changing profile changes the contract; it is not a shortcut for fixing a dimension error.

**If it doesn't**: on ``real_robot.cameras must match the camera_keys mapping``, align both name lists.
On ``rlinf_franka requires a camera mapped to observation/image``, fix the main-view mapping.
If force/torque feedback is missing or the reset reference differs, first supply the device capability and check the training definition.
If directions, scales, or gripper semantics differ, stop at this section; do not execute the next step.

8. Run one supervised episode and read the execution record
--------------------------------------------------------------------------------

Use the checked configuration for one short episode and save the terminal record to compare measured motion with the report.
The parameters below follow this repository's deployment example: 10 Hz, a 30-step cap, and at most 5 executed steps per inference;
``120`` seconds is this chapter's example cap for waiting on disconnection, not the program default.
First change camera names, prompt, scales, and frequency to the values verified in Sections 2 and 7.

.. warning::

   The next command resets, homes, opens the gripper, and executes policy actions. First confirm the home path, payload, workspace,
   per-step limits, and stop method, with on-site supervision and a physical stop device available.
   A passing dry-run and an online daemon do not replace these checks.

.. tutorial-test: skip
.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start episode
   :end-before: # end episode

**What you should see**: one synthetic ``warmup``, then reset, then a request for the first action chunk using real post-reset observations.
At most ``--replan-steps`` steps execute from each chunk; when unspecified, the default executes the whole horizon.
Below is an episode-output example using the ``loop.py`` format; the numbers are not hardware test results:

.. literalinclude:: ../../examples/en/deploy-openpi-output.txt
   :language: text
   :start-after: # start episode
   :end-before: # end episode

Read ``steps``, ``health_ok``, and ``outcome``, then compare ``rtt_ms_mean``,
``inference_ms_mean/max``, and ``server_infer_ms_mean``. Missing server timing appears as ``n/a``.
Reaching the step cap is truncation, not success; without a task-success criterion, finishing an episode does not mean a successful grasp.
The deployment loop's printed ``success_rate`` comes from the environment outcome; it does not recognize task results in video.

To end early, press Ctrl-C in the deployment terminal; the loop's cleanup path closes the environment and disconnects devices.
Still confirm on site that the arm actually stopped, and use the physical stop device for abnormal motion.
This entry point does not have the takeover-button workflow from the earlier teleop chapters.
Then read the record you just saved:

.. tutorial-test: skip
.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start log
   :end-before: # end log

``local/deploy-openpi.log`` saves summaries, timings, and disconnection events, not step-by-step action/observation trajectories.
If you need to compare policy and executed actions step by step when entering real-robot RL, arrange trajectory collection separately;
this chapter's command does not generate that archive. Do not treat a summary as the complete recording from :doc:`inspect`.

**If it doesn't**: if the first reset fails, recheck home, safety bounds, and device feedback. If watchdog
``interrupted`` events repeat, measure action intervals as in the next section. If ``health_ok=False``, stop and inspect daemon logs.
If actions differ from expectations, save the record and return to Section 7; do not enlarge boundaries to keep trying.

9. Disconnection, waiting, and recovery
--------------------------------------------------------------------------------

Understand the waiting state before maintaining the tunnel so you do not mistake a restored connection for continued execution of the old action chunk.
``WAITING`` means the deployment loop is waiting for the robot link or operator, not that the policy is still completing the task.
It occurs on a recoverable transport error or recoverable daemon watchdog fault within an episode.
Policy WebSocket errors, safety errors, and unrecoverable configuration errors exit instead of entering this robot-reconnection flow.
A connection failure during initial environment creation is not guaranteed to enter WAITING either.

When the tunnel drops, the NUC watchdog stops the servoing arm after its timeout and enters IDLE.
The loop discards the current action chunk and prefetched results, keeps the policy connection, and retries with backoff at 1, 2, 4, 8, capped at 10 seconds.
Reconnection rereads the device description, negotiates capabilities by daemon identity, checks arm health, and handles reported recoverable errors.
The source-code format is illustrated below:

.. literalinclude:: ../../examples/en/deploy-openpi-output.txt
   :language: text
   :start-after: # start waiting
   :end-before: # end waiting

**What you should see**: a confirmation prompt after reconnection. Inspect the site before pressing Enter; the program resets and starts a new episode,
rather than continuing midway through the old action chunk. The interrupted attempt counts in ``interrupted_episodes``,
not in completed episodes or the success-rate denominator; ``disconnects`` and ``disconnected_s`` retain the disconnection record.
The interruption duration starts at the episode's last successful robot interaction, or at the error if there has not yet been a successful interaction.

Without interactive stdin, it prints ``WAITING for operator confirmation (no interactive stdin)`` and waits.
``--auto-resume`` skips confirmation and allows automatic reset after reconnection; enable it only when automatic reset is explicitly permitted on site.
Run deployment in a reattachable GPU terminal such as tmux to keep the manual confirmation input available.

``--reconnect-timeout-s`` has no limit by default; Section 8 sets it to 120 seconds.
It starts when WAITING begins, limits retries and noninteractive-stdin waiting, and exits with device-error code 2 on timeout.
**Implementation boundary**: this deadline does not interrupt interactive ``input()`` waiting for Enter;
nor does it forcibly abort a device call already in progress. It is therefore not an exact total wall-clock limit.

Keep the tunnel alive independently of the terminal
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

First install autossh on the GPU and configure SSH key authentication, then put the service under user systemd.
The content below is generated by this repository's ``link_host.tunnel_service``, using the same ports as this chapter's manual tunnel:

.. literalinclude:: ../../examples/deploy-openpi.service
   :language: ini
   :start-after: # start tunnel-service
   :end-before: # end tunnel-service

.. tutorial-test: skip
.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start service
   :end-before: # end service

Put the path from ``command -v autossh`` into ``ExecStart``; for example, output of ``/usr/bin/autossh``
exactly matches the path above. Keep the username and NUC address from Section 5. Stop the manual tunnel before starting the service
to avoid contention on 50061. ``enable-linger`` keeps it running after logout; complete this according to the host's permissions.

**What you should see**: the user service is active and GPU doctor's ``link.daemon`` passes again.
CLI-only doctor uses ``rwrl-tunnel-50061.service``; with a station ``link:``
configuration, the service name is generated from the robot name. Read doctor's proposed service name and plan before using ``--fix``.

**If it doesn't**: if ``link.tunnel`` reports systemd status unavailable, check in a user session on the GPU host.
If the service keeps restarting, inspect SSH keys, host fingerprints, and port ownership. If ``link.forward`` fails, fix NUC forwarding configuration.
autossh restores only the tunnel; it neither confirms reset for the deployment loop nor replaces the watchdog.

Allow measured time for the watchdog
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Read the Section 8 logs and ``link.quality`` before choosing the NUC's ``--watchdog-s``; do not copy 2 seconds blindly.
After each reset, roughly three daemon round trips occur before the first action, plus policy inference;
also include image transfer, tunnel jitter, the control period, and actual device-call time.
The current station-built franky Hand backend submits open/close commands asynchronously. Do not estimate timing from the old blocking Hand
implementation; measure actual action intervals with the current backend.
For example, at the 400 ms RTT discussed in this repository's documentation, three round trips already take about 1.2 seconds, so a 1-second timeout repeatedly interrupts newly recovered episodes.

**What you should see**: the chosen value covers the longest normal on-site action interval with margin, while the stop delay after disconnection remains acceptable.
The ``link.quality`` estimate excludes inference, so add measurements from the real service.
If you also enable the environment's ``real_robot.watchdog_s``, check it against the same budget.

**If it doesn't**: if episodes remain ``interrupted``, first reduce transfer load, fix the proxy route, and check control frequency;
do not increase the timeout indefinitely. ``--prefetch-steps`` defaults to 0; enabling it overlaps inference, but the next action chunk uses an older observation,
which needs separate evaluation. Keep it disabled for this chapter.

Handle Clash / mihomo fake-IP
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If ``link.proxy`` reports that loopback addresses do not bypass the proxy, set the environment according to its actual hint and rerun doctor:

.. tutorial-test: skip
.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start proxy
   :end-before: # end proxy

If it reports the SSH host resolving into ``198.18.0.0/15``, doctor gives two routes:

* Query the real address on the NUC, then set ``HostName`` for that target in the GPU's SSH configuration.
  Keep ``HostKeyAlias`` as instructed by the hint so the original known_hosts association is preserved.
* Add the hostname to ``dns.fake-ip-filter`` in Clash/mihomo and place that domain's DIRECT rule first;
  or exclude the NUC's real address with ``tun.route-exclude-address``.

.. tutorial-test: skip
.. literalinclude:: ../../examples/en/deploy-openpi.sh
   :language: bash
   :start-after: # start real-ip
   :end-before: # end real-ip

**What you should see**: after the queried real address is confirmed on site, the new SSH path no longer uses fake-IP;
remeasure RTT and image throughput with ``link.quality``. Setting only ``no_proxy`` does not fix SSH's DNS path.

**If it doesn't**: on ``Host key verification failed``, verify the fingerprint and ``HostKeyAlias``.
If DNS queries still return a fake address, fix local resolution first. If proxy warnings disappear but RTT remains high, keep checking the link and image sizes.
These are this repository's doctor diagnostic suggestions; this chapter has not validated their effect on a real Clash/NUC network.

Further reading: bimanual Clean Desk
--------------------------------------------------------------------------------

Read bimanual ``Clean Desk`` as a contract extension: even with Frankas, it adds left/right arm layout and synchronization issues.
Use Section 1's inspection method to list its training configuration. Also check the left/right state concatenation order, each reset reference frame,
which arm each view corresponds to, both gripper encodings, and how one action chunk splits across two devices.
Confirm bimanual timing, both home paths/boundaries, and the collision space between arms too.

**What you should see**: an input/output table explicit about “which dimension belongs to which arm, and where each image comes from.”
**Needs verification**: Clean Desk's upstream weights, loader, normalization statistics, exact dimensions, and camera layout.
These configurations are not included locally; simply doubling the single-arm 19/7 dimensions is not a confirmed contract.

**If it doesn't**: this chapter's ``deploy-openpi`` constructs a single ``RealRobotEnv``;
neither existing profile can directly claim support for this bimanual policy. A separate bimanual adapter must be implemented and verified.
Do not feed bimanual actions into the current single-arm entry point.

.. note::

   **What needs hardware confirmation in this chapter**

   WebSocket loading and inference with actual Pick_Red weights remain unverified. Also confirm real camera views,
   crop and colors, state/torque and stiffness frames, action directions, scales and gripper thresholds, home and workspace,
   inference and transfer timing, watchdog stopping, and manual confirmation and reset after disconnection.
   autossh, systemd, and Clash routing need validation on both on-site hosts; Clean Desk also needs bimanual contract and synchronization validation.
   Local results in this chapter cover only the CLI, fake-policy protocol, and diagnostics without a service; they do not constitute hardware acceptance.

Checklist: before real-robot reinforcement learning
--------------------------------------------------------------------------------

* The actor's observation and action configuration is verified; this chapter's profile, cameras, scales, and station have been checked item by item.
* You know how the policy produces action chunks from observations and how many steps execute each time; the actual Pick_Red WebSocket inference verification item has a result.
* You know how to stop the current episode and read execution summaries and disconnection records, and know that summaries do not contain step-by-step action trajectories.
* Real-robot reinforcement learning (starting at :doc:`rl-overview`) reuses this station and verified configuration; those chapters explain HIL-SERL policy initialization separately.

Continue to :doc:`rl-overview` in the next chapter.
