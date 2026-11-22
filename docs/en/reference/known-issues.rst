.. _en-known-issues-known-issues-and-deferred-work:

Known issues and deferred work
================================================================================

What a first real run on one Franka arm with a gripper can run into, where it is in the code, and
what to do about it today. The tutorial's default route runs the hardware process directly on the NUC
with GELLO as ``--teleop-device leader_cartesian`` and keys from ``--teleop-event-device stdin``. The
robot-daemon route (``--teleop-device leader_arm``, keys from ``--teleop-event-device keyboard``) is also
supported; entries name the teleop device they affect. Every ``path:line`` was checked against the code when the
entry was written; line numbers drift, so search for the named function if one no longer matches.
Numbers marked "static analysis, not measured" come from reading the code, not from a run.

Each entry gives the symptom, the impact, where it is, the workaround for today, the planned fix with
the test that will prove it, and its status. Fixed entries move to :ref:`Closed <en-known-issues-closed>` with their validation.

.. _en-known-issues-a-to-fix-after-the-first-real-run:

A. To fix after the first real run
--------------------------------------------------------------------------------

In dispatch order.

.. _en-known-issues-s8-untrained-joint_abs-startup-motion:

S8. Untrained ``joint_abs`` startup motion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A normalized action near zero commands the midpoint of joint limits. An initial learner
snapshot can still contain an untrained policy; the readiness handshake does not replace
that policy with a measured-pose hold for nonresidual ``joint_abs`` runs. Commission this mode
with the configured per-step motion bounds. A hold/warm-up policy before autonomous control
remains open. Truncation/bootstrap boundaries now have regression coverage in
``tests/test_partial_imitation.py`` and the trajectory recovery tests.

Status: open for the startup hold/warm-up behavior (S8).

.. _en-known-issues-s3-joint-space-leaders-taking-over-can-open-the-gripper:

S3 (joint-space leaders). Taking over can open the gripper
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  **Symptom**: with ``--teleop-device leader_arm``, ``gello`` or ``robot``, the policy holds the object,
   you take over (``t``) with the trigger released, and the gripper opens at once.
-  **Impact**: safety (a dropped object) and a bad intervened row.
-  **Where**: the trigger maps straight to ``2w - 1`` (``TeleopPartSource._gripper_action`` in
   ``real_world_rl/teleop/core/part.py``); the onset policies act on the joints only
   (``LeaderArmPartSource._poll`` in ``real_world_rl/teleop/leader/joint.py``).
-  **Workaround today**: before taking over, set the trigger to the gripper's current state
   (squeezed if it holds the object). To learn which trigger positions read as open and closed,
   run ``rwrl teleop read`` before the session (no robot motion; not while a recorder or actor holds
   the leader): it prints the trigger live as ``gripper=``.
-  **Planned fix**: the take-over latch ``leader_cartesian`` has (see :ref:`Closed <en-known-issues-s3-leader_cartesian-taking-over-can-open-the-gripper>`\ ).
-  **Status**: open for the joint-space leader sources.

.. _en-known-issues-zero-twist-holds-left-on-cartesian_delta:

Zero-twist holds left on ``cartesian_delta``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  **Symptom**: the arm drifts (an impedance backend re-anchors its equilibrium at the measured
   pose) where a hold still sends a zero twist instead of the fixed hold pose.
-  **Where**: the intervention wrapper's fill-in for the dims the operator does not drive
   (``--teleop-selected-parts`` with ``--teleop-idle-behavior hold``, or a teleop part without a
   gripper; ``InterventionWrapper`` calls the hold without its ``reset()``/``observe_step()`` hooks), the
   classifier-frame collector (``real_world_rl/reward/collect_real_robot.py``), and two-arm
   ``cartesian_delta`` envs (``hold_action_fn_for`` keeps the zero arm action for ``num_arms > 1``).
-  **Workaround today**: record demos with the recorder (its hold is fixed); keep the arm under
   the operator while collecting classifier frames on a cartesian arm.
-  **Planned fix**: the wrapper and the collector drive the hold's hooks; one tracker per arm for
   two-arm envs.
-  **Status**: open.

.. _en-known-issues-docs-stale-items-in-hil-serl-roadmapmd:

Docs: stale items in ``hil-serl-roadmap.md``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Status: fixed in the :doc:`RTD roadmap <hil-serl-roadmap>`. Its current-status notes now
cover the demo recorder, leader-arm and dual-arm support, durable recovery, and the
``HilSerlArgs`` entropy/augmentation/pooling defaults. Historical migration descriptions
remain identified as historical.

.. _en-known-issues-b-not-planned:

B. Not planned
--------------------------------------------------------------------------------

-  **GELLO torque-follow** (the leader tracking the follower): needs Dynamixel current control in
   the device layer and an operator-safety review; the ``ramp`` and ``arm`` onsets cover the takeover
   (\ :ref:`gello.md <en-gello-9-future-work-leader-tracking-wp8>`\ ). Re-evaluate after the GELLO latency has
   been measured on hardware.
-  **Handing back by letting go of the leader**: there is no grip sensor, and a pause would hand back
   by mistake. Use ``h`` (``--teleop-event-device keyboard`` or ``stdin``), or a foot pedal through the
   GELLO's ``event_device`` option (\ :ref:`teleop.md <en-teleop-leader-arms-gello>`\ ).
-  **The actor stops on a teleop read fault**: by design. A failed read propagates
   (``real_world_rl/teleop/core/contract.py:168``, polled before every step at
   ``real_world_rl/envs/wrappers/intervention.py:214``), so a dropped reading is never mistaken
   for a policy step. Actor cleanup now closes the environment; a forced kill still relies on the watchdog.

.. _en-known-issues-c-unverified-on-hardware:

C. Unverified on hardware
--------------------------------------------------------------------------------

Everything hardware-side has run against fakes only, ``leader_cartesian``'s forward kinematics has not
been compared with a robot, ACT has not been trained on real pkls, and the reward classifier has not
seen real frames. Details: :doc:`Overview, Known limits <../index>`\ .

.. _en-known-issues-closed:

Closed
--------------------------------------------------------------------------------

.. _en-known-issues-cartesian-hold-drift-cartesian_delta-control-v2:

Cartesian hold drift (``cartesian_delta``, control v2)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``RealRobotEnv`` in ``cartesian_delta`` sends ``measured + action * scale``; with an impedance backend a
zero hold action re-anchored the equilibrium wherever the arm drifted (137 mm in 24 s on the first
hardware session). The recorder's hold now keeps a fixed hold pose (sampled at the env reset and at
every hand-back), and ``leader_cartesian`` drives a persistent command target that advances at most
0.05 m/s / 0.3 rad/s (``--teleop-target-speed-m-s``/``--teleop-target-speed-rad-s``) and leads the
measured TCP by at most 0.05 m / 0.3 rad (``--teleop-lead-cap-m``/``--teleop-lead-cap-rad``; the excess
is dropped). The env scales must be at least the caps. The experiment contract records it as
``teleop_control`` (``cartesian_tracker_v1``). See
:ref:`teleop.md <en-teleop-leader-arm-on-a-cartesian-robot-leader_cartesian>`\ .

The paths that still send a zero twist are listed above (Zero-twist holds left on
``cartesian_delta``).

Status: fixed for the recorder and ``leader_cartesian`` (offline only, not yet re-run on the arm).
Regression coverage: ``tests/test_cartesian_tracker.py``, ``tests/test_leader_cartesian_source.py``,
``tests/test_real_demo_recorder.py``.

.. _en-known-issues-s3-leader_cartesian-taking-over-can-open-the-gripper:

S3 (``leader_cartesian``). Taking over can open the gripper
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

At the take-over, a trigger that disagrees with the gripper's last commanded state is ignored: the
source keeps the last commanded gripper action (the intervention wrapper's
``last_gripper_action_getter``) until the trigger crosses to that state.

Status: fixed for ``leader_cartesian`` (and for ``spacemouse``, below); the joint-space leaders are
still open (above). Regression coverage: ``tests/test_leader_cartesian_source.py`` (take-over gripper
latch).

.. _en-known-issues-s3-spacemouse-taking-over-can-open-the-gripper:

S3 (``spacemouse``). Taking over can open the gripper
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The old SpaceMouse source reset its sticky gripper to open at every env reset, so taking over
while the policy held an object sent ``+1`` and dropped it. ``--teleop-device spacemouse`` now starts
every tracker-mode take-over (button 0) from the gripper's last command (the intervention wrapper's
``last_gripper_action_getter``, kept as is, a continuous value included) and only button 1 changes
it, toggling between ``+1`` and ``-1``; button 1 does nothing while handed back.

Status: fixed for ``spacemouse`` (offline only, not yet run on the arm). Regression coverage:
``tests/test_spacemouse_teleop_source.py`` (take-over gripper, continuous last action, button 1
without a take-over). Not fixed on the robot-yaml route: ``--teleop-device robot`` with a
``backend: spacemouse`` teleop section keeps the old semantics (raw twist, moving the puck
intervenes, button 0 toggles a sticky gripper that opens at every reset), as does the sim-only
``spacemouse_twist``.

.. _en-known-issues-s1-fwbw-intervention-tracing:

S1. FWBW intervention tracing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each FWBW direction now emits its own durable trajectory segment with executed/proposed
commands and actual human masks. Human rows reach online and demonstration replay through
the same training projection as other actor paths; segment boundaries prevent n-step
windows crossing a direction change.

Status: fixed. Regression coverage:
``tests/test_p1_ingestion.py`` FWBW trace/partial-human regression.

.. _en-known-issues-actor-parameter-versions:

Actor parameter versions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Actor snapshots use learner session/version identity. The actor loads a new state only when
that identity changes, including a learner restart with a lower version in a new session.
Status: fixed. Regression coverage: P0 lifecycle tests.

.. _en-known-issues-s2-catch-up-motion-and-partial-demonstration-labels:

S2. Catch-up motion and partial demonstration labels
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The catch-up after a takeover carries syncing metadata: in ``joint_delta`` the steps until the leader
is first within ``joint_scale_rad`` of the follower, in ``joint_abs`` the rate-limited ramp steps. Once
aligned, a takeover is supervised normally. Durable archives preserve the executed motion and
actual human mask; the training projection excludes syncing supervision. Human/override masks
also keep hold supplements out of partial imitation losses. Recorder pickle exports made with
``--keep-syncing-steps`` keep those rows as ordinary supervised steps.

Status: fixed. Regression coverage:
``tests/test_partial_imitation.py``, ``tests/test_leader_arm_source.py``, and recorder regressions.

.. _en-known-issues-s4-learner-replay-and-checkpoint-recovery:

S4. Learner replay and checkpoint recovery
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Durable journal admission, atomic checkpoint generations, replay-extension restoration and journal cursors now preserve committed rows and avoid bundle-plus-snapshot double loading. Legacy native replay remains readable with explicit provenance limits.

Status: fixed. Regression coverage: ``tests/test_replay_state.py``, ``tests/test_p0_process_recovery.py``.

.. _en-known-issues-s6-actor-lifecycle-and-pending-uploads:

S6. Actor lifecycle and pending uploads
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Actor/eval cleanup releases environment resources and drains uploads with a deadline. SQLite outboxes retain unacknowledged messages and SiLRI held rows. A forced process kill still relies on the daemon watchdog; the last action before its durable commit is outside the persistence guarantee.

Status: fixed. Regression coverage: ``tests/test_p0_process_recovery.py`` and ``tests/test_p0_lifecycle.py``.

.. _en-known-issues-s5-silri-pretraining-and-ingestion:

S5. SiLRI pretraining and ingestion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

HTTP admission commits to the learner journal independently of the training buffer lock. Stable transition IDs make retries idempotent; outbox rows survive a missed ACK or process restart.

Status: fixed. Regression coverage: ``tests/test_p0_process_recovery.py`` and ``tests/test_p0_lifecycle.py``.

.. _en-known-issues-s7-autonomous-and-assisted-evaluation:

S7. Autonomous and assisted evaluation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Shared explicit outcomes now drive actor/eval/deploy/recording. The primary success rate uses all completed episodes and credits only successes without action intervention; total task success and intervention rates are separate. Positive reward or termination alone does not imply success.

Status: fixed. Regression coverage: ``tests/test_episode_outcomes.py``, ``tests/test_p0_lifecycle.py`` and ``tests/test_real_demo_recorder.py``.

Operational details and remaining recovery limits: :doc:`Real-world experiments <real-world-experiments>`\ .
