Understand an online RL experiment
================================================================================

These real-robot RL chapters connect the workstation from :doc:`station`, the demonstrations from :doc:`record`, and the execution checks from :doc:`deploy`
into one online experiment. Complete the deployment chapter's observation, action, and stop checks before letting a policy control real hardware.
If you use openpi, also complete :doc:`deploy-openpi` first. These chapters train this repository's HIL-SERL policy;
they do not automatically turn an openpi service into HIL-SERL's initial policy.

These chapters use the single arm, seven-dimensional Cartesian actions, and manual success/failure labels verified earlier.
SpaceMouse uses tracker; GELLO uses leader_cartesian. Reuse ``local/station.yaml``,
``local/task.yaml``, and ``local/first-demo.pkl``. Write a separate ``local/rl-task.yaml`` for training settings.
Leave the classifier, grasp penalty, and reset-free FWBW disabled for now, so you can check exactly where each reward comes from.

1. Distinguish the three processes
--------------------------------------------------------------------------------

Place the processes as shown below so the learner and actor do not both open the hardware.
The device service owns the arm, gripper, and camera connections; the actor runs the control loop;
the learner updates policy parameters. These chapters place the actor and device service on the robot PC and the learner on the training machine.
Both can also run on one machine; use a loopback address to connect to the learner in that case.

.. literalinclude:: ../../examples/en/rl-overview-diagram.txt
   :language: text
   :start-after: # start topology
   :end-before: # end topology

The actor finds the device service through ``real_robot.remote`` and the learner through ``sync_host:sync_port``.
These addresses use different ports. The training machine only constructs observation/action spaces. It does not reset or step the real arm,
or open teleop devices. Supply camera sizes in the configuration; the learner cannot infer them by connecting to a camera.

.. warning::

   Watchdogs have failure boundaries. A watchdog thread in a direct-control process cannot cover a freeze of the entire process.
   The separate ``rwrl supervise`` can terminate its child process; the direct connection then relies on firmware to handle the lost control connection.
   The daemon watchdog can stop a servoing arm when the client stops sending actions, but it still runs inside the daemon process
   and cannot cover a daemon or NUC failure. Keep on-site supervision and a working physical stop device available.

Inspect the entry points in each machine's installed environment without connecting to devices:

.. literalinclude:: ../../examples/rl-train.sh
   :language: bash
   :start-after: # start help
   :end-before: # end help

**What you should see**: train lists ``hil_serl``. Its help includes ``--role {actor,learner,eval}``,
``--sync-host``, ``--sync-port``, ``--actor-outbox-path``, and ``--load-checkpoint``.
All three roles share one method entry point; ``examples/train_real_world.py`` calls the same registry.

**If it doesn't**: if rwrl is missing, return to :doc:`install` and activate the installed environment. In a source environment,
use the module entry point in :doc:`rl-train`. Setting sync_port to 50051 connects to the wrong service. If you gave the actor
the learner's 0.0.0.0 bind address, replace it with the training machine's reachable address.

2. Follow the data through one time step
--------------------------------------------------------------------------------

Choose a record produced in the next chapter and check its inputs and outputs in this order.

#. The actor waits for the learner's initial snapshot before reset. reset reads state and cameras from the device service to produce the first obs.
   Each later step's obs comes from the previous step's next_obs.
#. At the start of each step, the actor checks for a new snapshot, then proposes ``policy_action`` with the current policy.
   HIL-SERL samples actions stochastically by default; eval uses deterministic prediction.
#. The teleop wrapper selects the executed action from the takeover state and per-dimension mask. The policy controls when nobody takes over;
   during human takeover, ``intervene_action`` replaces the corresponding actions. The device layer still applies limits and safety constraints.
#. The environment sends the action to the device service, then reads next_obs. Manual ``s``/``f`` events produce the task outcome;
   here success gives reward 1 and terminates the episode, while failure gives reward 0 and truncates it.
#. The actor writes obs, action, reward, next_obs, and done, and also archives policy_action,
   human_action_mask, episode markers, and the policy version. ``action`` is the request sent to the environment,
   not the displacement measured by encoders; compare observations before and after to check actual motion.
#. After committing to the local outbox, the actor uploads asynchronously. The outbox is a SQLite queue of pending records; the learner's journal
   is the SQLite log written first when records arrive. An HTTP ACK means the journal received the record, not that it has been used for training.
#. The learner adds records to online replay in journal order; human samples that are not syncing also enter demo replay.
   It samples a mixture from both replays, updates, and publishes parameters. The actor can load the new version at the next step.

Use the same inspection script to read records. After starting the processes in Chapter 3, run this on the learner machine:

.. tutorial-test: skip
.. literalinclude:: ../../examples/rl-train.sh
   :language: bash
   :start-after: # start inspect
   :end-before: # end inspect

**What you should see**: three record types: ``episode_start``, ``transition``, and ``episode_end``.
A transition has ``record_id``, ``episode_id``, ``step_index``, and action/policy-version fields.
Chapter 4 uses the same record_id to find a human correction. An episode may still be running; do not treat the current last row as a complete episode.

**If it doesn't**: if the file does not exist yet, complete Chapters 2 and 3 first. An empty outbox does not mean nothing was collected: its rows are deleted after ACK,
so read the learner journal. If you see only episode_start, check the actor's first step, error logs, and upload status;
START_NEUTRAL waits before episode_start is written.
truncated without terminated means truncation; it does not imply success.

3. Define the acceptance evidence for these chapters
--------------------------------------------------------------------------------

After each chapter, keep files or IDs that connect it to the next chapter.

* Task: success and failure are distinguishable in episode_end.outcome, and the demonstration contract matches the online configuration.
* Training: the journal contains transitions, learner/received and learner/update increase, and global_update
  in later actor records is greater than 0.
* Correction: one record_id accounts for policy_action, action, and human_action_mask;
  the learner has received that record, and the human-data count increases.
* Recovery: the matching journal and bundle are restored together, and independent eval reports task_success_rate and
  autonomous_success_rate.

**What you should see**: a continuous chain of evidence. A falling loss alone does not show that the real robot learned the task.
**If it doesn't**: if step-by-step data is missing, check the journal; if success labels are missing, check the event source; if policy versions are missing, confirm you are reading
the current actor's online archive, not a recorder file from :doc:`record`.

Distinguish “implemented/software checked” from “passed on the current hardware station.” A test checklist in the repository is not itself a passing test record.
The evidence currently available is bounded as follows; old station sessions and fake-environment tests do not establish hardware acceptance for every method.

.. list-table:: Evidence boundaries by workflow
   :header-rows: 1

   * - Workflow
     - Existing evidence or check materials
     - Still required on site
   * - Single-arm handshake, GELLO/SpaceMouse recording
     - Earlier sessions recorded hardware issues and fixes; follow-up acceptance checklists exist
     - Recheck direction, gripper, stopping, and archiving with the current software, controller, and station
   * - Bimanual
     - A bimanual workflow and reference test checklist exist; the checklist explicitly does not mean hardware has passed
     - Verify simultaneous reset, takeover, stopping, and each arm's workspace
   * - BC/ACT
     - Offline training and policy-contract check workflows exist; earlier records have not verified ACT training on real pkl data
     - Real-data training results and complete hardware deployment acceptance
   * - HIL-SERL/residual/SiLRI online learning
     - Methods are implemented; this tutorial's CPU fake-environment checks cover only the data and update chain
     - The full online hardware loop, human takeover, recovery, and independent evaluation

See :doc:`../reference/testing/index` for on-site checks and evidence preservation;
see :doc:`../reference/testing/bimanual-gello-session` for bimanual materials.
Do not mark hardware as passed without the corresponding configuration, logs, and measured records.

.. note::

   **What needs hardware confirmation in this chapter**

   Confirm device-service connections, observation freshness, actions versus measured motion, takeover and handback, stopping, and control timing on site.
   The CPU fake environment here verifies only the data and update chain; the sandbox blocked local sockets, so HTTP synchronization still needs on-site verification.

Checklist: identify both service addresses; know who sends actions, who writes rewards, and who performs gradient updates;
distinguish requested actions, measured motion, journal receipt, and replay training.

Next: :doc:`rl-task`.
