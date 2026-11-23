.. _en-hil-serl-roadmap-hil-serl-roadmap:

HIL-SERL Roadmap
================================================================================

The franka_real env was removed; real_robot + serl_http replaces it.

This is a historical porting record. For current operation use the :doc:`tutorial <../index>`\ ,
:doc:`Real-world experiments <real-world-experiments>` and :doc:`Known issues <known-issues>`\ .
The current implementation includes the demo recorder, leader-arm and dual-arm support,
durable outboxes/journals, replay bundles and explicit autonomous/assisted outcomes. The item
descriptions below retain the migration-era behavior; they are not the current support matrix.

This document moved from rl-garden when the real-world layer became this repository.
Current local paths use ``real_world_rl/``; ``rl_garden/`` and ``3rd_party/`` paths
below are relative to the ``rl-garden/`` submodule, including its vendored HIL-SERL
reference. The old ``robot_infra`` device package is now ``real_world_rl``; old
controllers and teleop live under ``real_world_rl/legacy/``.

Since this document was written the real-world layer also gained the
``TeleopSource`` registry, a shared HIL env builder, an overridable actor-loop
step, an ``InterventionReplayBuffer``, and the ``residual_hil_serl`` and ``silri``
methods; see "Landed since the initial migration" at the end of the
"Not Yet Migrated" list. Sections below keep their original wording where
they describe the state at the time of the item.

This document tracks what HIL-SERL (``3rd_party/hil-serl/``) capability was
**not** migrated in the initial ``hil_serl`` training-method landing (see
``real_world_rl/training/entrypoints/hil_serl.py``), and why. HIL-SERL itself splits
into three largely independent training scripts -- ``train_rlpd.py`` (online
RLPD + demo mixing + HITL + reward classifier), ``train_bc.py`` (standalone
BC), and ``train_hgdagger.py`` (BC pretrain + iterative correction against a
growing on-disk dataset) -- and this round targeted only the
``train_rlpd.py`` capability set.

Items marked LANDED record completed migration work. Current operation additionally
includes demo recording, dual-arm control and durable recovery; the current-status
notes below supersede the older gap descriptions.

A follow-up source-level comparison against ``3rd_party/hil-serl/examples/train_rlpd.py`` (the only authoritative HIL-SERL implementation) surfaced one
concrete bug in the just-landed item 7, plus three additional gaps not
previously tracked here -- see "Item 7 correction" and items 8-10 below (11
remains open).

A second, broader comparison round split HIL-SERL's own code-structure table
(``3rd_party/hil-serl/README.md``'s "Overview and Code Structure") into its 7
leaf categories -- ``examples``, ``serl_launcher.{agents,data,vision,wrappers}``,
``serl_robot_infra.{robot_servers,franka_env}`` -- and diffed each against the
current ``rl-garden`` implementation. It re-confirmed items 1-11 below are
still accurate, sharpened several of them (see "Item 8 correction" and the
caveats added to items 2, 5, 9, 10, and 11), and surfaced a substantial batch
of previously untracked findings: one concrete correctness bug (item 12), a
set of missing capabilities (items 13-20), and a class of "implemented, but
the default/behavior differs from what HIL-SERL's own real-robot recipes
actually use" divergences, tracked separately in "Behavioral Divergences
from HIL-SERL's Real-Robot Defaults" below.

.. _en-hil-serl-roadmap-item-7-correction---offline_dataset-silently-discarded-by-init_demo_buffer:

Item 7 correction: ``--offline_dataset`` silently discarded by ``init_demo_buffer``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``DemoInterventionMixin.init_demo_buffer()`` (``rl_garden/buffers/demo_intervention.py``) unconditionally replaces ``self.offline_replay_buffer``
with a fresh empty buffer. ``_run_learner`` (``real_world_rl/training/entrypoints/hil_serl.py``) calls ``build_rlpd_hybrid(args, ...)`` (which loads
``args.offline_dataset`` into ``offline_replay_buffer`` if set) and *then*
``agent.init_demo_buffer(...)`` -- so a user who passes ``--offline_dataset``
to ``hil_serl`` doesn't get a "can't use both" error, they get their loaded
data **silently discarded** with no diagnostic. Item 7's original text framed
this as an unsupported combination; it's actually a data-loss bug. (Now fixed
-- see item 8 below.)

.. _en-hil-serl-roadmap-8-demo-data-preloading---demo_path-equivalent----landed:

8. Demo-data preloading (``--demo_path`` equivalent) -- LANDED
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Current full-bundle recovery restores the saved demo buffer (including retained seed demos), so it does not preload seed files again. ``--load-actor-checkpoint`` is rejected for hybrid policies: the BC continuous action head does not match the arm actor plus DQN gripper.

HIL-SERL's own ``--demo_path`` (``train_rlpd.py:458-466``) is not RLPD's generic
``--offline_dataset`` (a structured h5/minari dataset loader) --
it's a list of raw pickled transition dicts, unconditionally loaded into the
same ``demo_buffer`` that intervention data grows into, on every process start
*including restarts* (loaded again each time, then crash-recovery snapshots
are glob-reloaded on top). Before this round, ``rl-garden``'s demo buffer had
no preloading path at all -- ``init_demo_buffer()`` always started empty, so a
``hil_serl`` run could only ever bootstrap from human intervention accumulated
during that run, with no way to seed it from prior teleop demonstrations (the
way real HIL-SERL runs almost always start).

Now built:

-  ``HilSerlLearnerLoop.__init__`` gained a ``demo_dataset_paths: Sequence[str]``
   parameter (glob patterns), loaded via ``_load_demo_datasets()`` into
   ``agent.offline_replay_buffer`` (through ``add_demo_transition``) when no full bundle
   is available, before legacy snapshot reload. Full bundles restore the saved demo buffer.
-  File format matches the existing buffer-snapshot pkl convention (a pickled
   list of ``{"obs", "next_obs", "action", "reward", "done", ...}`` CPU-tensor
   dicts -- the same shape ``HilSerlLearnerLoop._snapshot()`` writes), not
   HIL-SERL's own numpy ``{"observations", "actions", ...}`` dict shape. This is
   a deliberate format choice, not a fidelity gap: it means a previous run's
   ``checkpoint_dir/demo_buffer/*.pkl`` snapshot files can be reused directly as
   another run's ``--demo_dataset_paths`` seed data.
-  New ``HilSerlArgs`` field (now tuple-typed for YAML presets): ``demo_dataset_paths: tuple[str, ...] = ()``.
-  ``DemoInterventionMixin.init_demo_buffer()`` now raises if
   ``offline_replay_buffer`` is already populated (e.g. from
   ``--offline_dataset``) instead of silently overwriting it -- turns the
   item-7-correction bug above into an explicit error. The underlying
   limitation (RLPD's static offline-dataset slot and HIL-SERL's demo-buffer
   slot can't both be used on one ``RLPDHybrid`` instance) is unchanged, just no
   longer silent.

**Item 8 correction** (fixed; the paragraph records the original finding): ``_load_demo_datasets()`` (called from
``HilSerlLearnerLoop.__init__``, before ``_reload_snapshots()``) increments
``self._received`` for every preloaded demo transition, even though those only
reach ``agent.add_demo_transition`` (the demo buffer), never
``agent.replay_buffer`` (the online buffer). ``received_transitions`` gates
``LearnerLoop.run()``'s wait-for-``learning_starts`` check
(``real_world_rl/loops/learner.py``). HIL-SERL's own analogous gate
(``while len(replay_buffer) < config.training_starts``, ``train_rlpd.py:278``)
checks only the online buffer's length -- its ``demo_buffer`` (populated from
``--demo_path``) never counts toward it. Concretely: a ``hil_serl`` run seeded
with ``--demo_dataset_paths`` supplying >= ``learning_starts`` transitions can
start training with **zero** real online transitions collected, which
HIL-SERL never allows (it always waits for genuine online data regardless of
demo/prior volume). ``tests/test_real_world_hil_serl_loops.py``'s
``test_hil_serl_learner_loop_preloads_demo_dataset_paths`` asserted
``received_transitions == 3`` after preloading with zero live transitions --
the test encoded this behavior rather than catching it. **Fixed**:
``_load_demo_datasets()`` in ``real_world_rl/loops/hil_serl.py`` no longer counts preloaded
demos toward ``received_transitions``, so ``learning_starts`` waits for online transitions as in
HIL-SERL, and the test now asserts ``received_transitions == 0`` after the preload.

**Now available**: ``rwrl record`` (``real_world_rl/demos/record.py``) writes complete
episodes into a demo dataset directory, and ``--demo-dataset-paths`` reads the human demonstrations of its kept episodes directly.
See :doc:`Recording demos <recording-demos>`.

.. _en-hil-serl-roadmap-9-real-world-agentpolicy-checkpointing----landed:

9. Real-world agent/policy checkpointing -- LANDED
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``LearnerLoop.run()`` (``real_world_rl/loops/learner.py``) used to drive
training via ``agent.train(gradient_steps)`` directly and never call
``OffPolicyAlgorithm._maybe_save_periodic_checkpoint``/``_save_checkpoint`` --
those are only invoked from ``OffPolicyAlgorithm.learn()``'s single-process
rollout loop, which no real-world entrypoint (``serl`` or ``hil_serl``) uses, and
that method also gates on ``_global_step`` (rollout-step count), which
``LearnerLoop`` never advances. HIL-SERL's own learner checkpoints full agent
state (weights + optimizer) every ``checkpoint_period`` *gradient-update* steps
and resumes via ``checkpoints.restore_checkpoint``
(``train_rlpd.py:249-355,419-429``).

Turned out most of the plumbing already existed: ``build_rlpd``/
``build_rlpd_hybrid`` (``rl_garden/training/online/entrypoints/{rlpd,rlpd_hybrid}.py``)
already call ``agent.load(args.load_checkpoint, ...)`` when ``--load_checkpoint``
is set, and both ``serl.py``/``hil_serl.py``'s ``_run_learner`` already pass
``args``/``checkpoint_dir`` straight through -- so **resume via**
``--load_checkpoint`` **already worked** for real-world learners, just untested
and undocumented. The only actual gap was the periodic-*save* trigger.

Now built, in the base ``LearnerLoop`` (shared by ``serl`` and ``hil_serl``, not
``HilSerlLearnerLoop``-only -- both bypass ``learn()`` identically):

-  ``BaseAlgorithm.global_update`` -- new read-only property exposing
   ``_global_update`` (restored correctly by ``load_state_dict``, so it stays
   monotonic across a resume, unlike a loop-local counter).
-  ``LearnerLoop._maybe_save_periodic_checkpoint()``, called after every
   ``_train_step()``: if ``agent.checkpoint_dir`` and ``agent.checkpoint_freq > 0``,
   saves ``checkpoint_{global_update}.pt`` once the gradient-update count
   crosses each ``checkpoint_freq`` multiple -- keyed on gradient updates,
   matching HIL-SERL's own ``step`` semantics in ``learner()``, not
   received-transition count.
   ``include_replay_buffer=agent.save_replay_buffer`` (existing flag, respected
   as-is).
-  ``LearnerLoop.run()``'s ``finally`` block now also does a one-shot ``final.pt``
   save when ``agent.save_final_checkpoint`` is set -- covers clean stops
   (``stop()``, ``KeyboardInterrupt``) in addition to the periodic saves that are
   the actual crash-recovery mechanism.
-  No new ``HilSerlArgs``/``SerlArgs`` fields needed -- ``checkpoint_dir``/
   ``checkpoint_freq``/``load_checkpoint``/``save_replay_buffer``/
   ``save_final_checkpoint`` were already exposed via the shared ``CheckpointArgs``
   (``rl_garden/common/cli_args.py``) and already flowed into the agent.

Known architectural consequence: ``tests/test_real_world_learner_loop.py``'s
documented "LearnerLoop only touches this minimal ``OffPolicyAlgorithm``
surface" contract grew to include ``checkpoint_dir``/``checkpoint_freq``/
``save_replay_buffer``/``save_final_checkpoint``/``global_update``/``save(...)`` --
an intentional, documented expansion (both ``_FakeAgent`` and the file's
docstring were updated), not scope creep.

**Caveat**: the cadence match to HIL-SERL claimed above is not exact.
HIL-SERL's ``checkpoint_period`` counts ``learner()``'s outer loop iterations
(``train_rlpd.py:314-355``), each of which bundles ``config.cta_ratio`` (default
2) total ``agent.update()`` calls -- one critic-only update plus one combined
critic+actor+temperature update. ``rl-garden``'s ``_global_update`` increments
once per raw gradient step inside ``sac_core.py``, and ``checkpoint_freq`` gates
on that finer-grained count. The same numeric ``--checkpoint_freq`` value
therefore triggers a save at a different real cadence on the two sides
(roughly 2x apart under HIL-SERL's default ``cta_ratio=2``) -- not a bug,
since both are internally consistent and the flag is ``rl-garden``'s own to
define, but worth knowing when comparing configured values across the two
codebases.

.. _en-hil-serl-roadmap-10-real-world-evaluation-mode----landed:

10. Real-world evaluation mode -- LANDED
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

HIL-SERL's actor supports ``--eval_checkpoint_step``/``--eval_n_trajs``: load a
specific saved checkpoint and run a fixed number of real-robot episodes to
measure success rate, no data collection or training (``train_rlpd.py:71-110``).

Now built as a third ``hil_serl``-only role (kept out of the shared
``RealWorldFrankaArgs.role`` to keep this round's blast radius limited to
``hil_serl``; ``serl.py`` is untouched):

-  ``HilSerlArgs.role: Literal["actor", "learner", "eval"]``, plus
   ``eval_n_trajs: int = 10``.
-  ``_run_eval(args)`` (``real_world_rl/training/entrypoints/hil_serl.py``): builds the
   env via ``build_hil_env`` (``training/env_building.py``, formerly ``_build_env``; same wrapper stack as actor/learner,
   including the reward classifier), builds the agent via ``build_rlpd_hybrid``
   -- requiring ``--load_checkpoint`` (raises ``ValueError`` if unset, unlike
   ``_run_actor``'s scratch-agent path which discards it) -- and runs
   ``eval_n_trajs`` episodes with ``deterministic=True`` fixed (ignores
   ``--deterministic_actor``, which governs the training-time actor's
   exploration instead). No sync client, no learner process, no data pushed
   anywhere. Current evaluation uses explicit outcomes, not ``terminated`` alone.
   Unknown completed outcomes remain in the denominator; interrupted/discarded
   attempts are excluded. Primary success credits only unassisted successes.
   See :doc:`Real-world experiments <real-world-experiments>`.
-  ``--fwbw`` + ``role="eval"`` raises ``NotImplementedError`` -- out of scope this
   round, same "single-arm only" boundary already documented elsewhere.

**Caveats found comparing against HIL-SERL's own eval loop**
(``train_rlpd.py:67-110``):

-  HIL-SERL's ``--eval_checkpoint_step`` mode is actually *stochastic*
   (``agent.sample_actions(..., argmax=False)``, ``train_rlpd.py:88-91``), despite
   being labeled "eval." ``rl-garden``'s ``role="eval"`` forces
   ``deterministic=True`` unconditionally -- stricter than what it's modeled
   on, not a bug, but it means the two sides' measured "success rate" isn't
   measuring the same policy behavior.
-  HIL-SERL's eval loop only checks ``done`` (``while not done:``,
   ``train_rlpd.py:86,95-98``), never ``truncated`` -- a timeout-only episode
   wouldn't end the reference loop. ``rl-garden``'s ``_run_eval`` explicitly
   breaks on ``bool(terminated) or bool(truncated)``; current outcome metrics
   use explicit labels rather than treating truncation as a failure label. Whether HIL-SERL's actual Franka envs also set ``done=True`` on
   timeout (which would make this moot in practice) was not verified.

.. _en-hil-serl-roadmap-11-grasp_penalty-reward-shaping-term----landed:

11. ``grasp_penalty`` reward-shaping term -- LANDED
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Built: ``real_world_rl/envs/wrappers/grasp_penalty.py::GraspPenaltyWrapper``
(ported from the task-specific ``usb_pickup_insertion/wrapper.py:: GripperPenaltyWrapper``, which is what HIL-SERL's own ``train_rlpd.py``/
``sac_hybrid_single.py`` actually consume via ``info["grasp_penalty"]`` -- not
the base ``franka_env/envs/wrappers.py::GripperPenaltyWrapper``, which mutates
``reward`` directly and is never wired into the real training loop) sets
``info["grasp_penalty"]`` on a gripper open/close flip. ``RLPDHybrid`` gained a
``use_grasp_penalty`` constructor flag; when set, its replay buffer stores a
``grasp_penalty`` column (``ReplayBuffer(store_grasp_penalty=True)``,
``rl_garden/buffers/replay_buffer.py``) and ``_train_discrete_critic``'s target
becomes ``(rewards + grasp_penalty) + ...``, matching HIL-SERL's formula.
``HilSerlActorLoop._extra_transition_fields`` forwards ``grasp_penalty`` from
``info`` to the pushed transition. New ``HilSerlArgs`` fields:
``use_grasp_penalty``, ``grasp_penalty_value``.

.. _en-hil-serl-roadmap-not-yet-migrated:

Not Yet Migrated
--------------------------------------------------------------------------------

.. _en-hil-serl-roadmap-1-hg-dagger-training-mode--the-growing-dataset-buffer-layer:

1. HG-DAgger training mode + the growing-dataset buffer layer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``train_hgdagger.py``'s iterative correction loop needs a continuously
growing, periodically-reloaded on-disk demo/correction dataset -- a
fundamentally different data path from ``LearnerLoop``'s current model (live
online transitions, or a static offline dataset loaded once via RLPD's
``offline_dataset``). This was flagged early and deliberately scoped out
because it isn't needed for ``train_rlpd.py``-style HIL-SERL.

-  ``LearnerLoop._refresh_offline_data()`` (``real_world_rl/loops/learner.py``)
   already exists as a no-op hook reserved for exactly this -- both ``serl``
   and ``hil_serl`` currently leave it unoverridden.
-  Reference design (from ``3rd_party/RLinf``'s real-world DAgger/SFT path,
   researched during this migration): a disk-file dataset pipeline fully
   decoupled from the in-memory RL replay buffer -- a LeRobot-format writer,
   a separate offline ETL step, and a lazily-reloaded PyTorch ``Dataset`` that
   picks up newly written data. Don't try to force growing demo data through
   the existing in-memory buffer interface.

.. _en-hil-serl-roadmap-2-bc-behavior-cloning-agent----landed:

2. BC (behavior cloning) agent -- LANDED
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Correction**: ``rl_garden/algorithms/bc.py::BC`` already existed and was
already fully registered (an ``OfflineRLAlgorithm``, pure NLL loss, no
critic) by the time this item's original text was written -- the actual gap
was only a real-world entrypoint, not the algorithm itself.

Now built: ``real_world_rl/training/entrypoints/bc.py::run_real_world_bc``. BC is
structurally an offline-style training loop over a static demo dataset --
unlike ``serl``/``hil_serl``, it has no actor/learner HTTP split at all (no
``sync.py``, no ``LearnerLoop``/``ActorLoop``), since ``BC`` is an
``OfflineRLAlgorithm``, not an ``OffPolicyAlgorithm``. Demo pkls are loaded via a
new shared helper, ``real_world_rl/training/demo_loading.py:: load_demo_pkls`` (factored out of ``HilSerlLearnerLoop._load_demo_datasets``,
which now calls it too instead of duplicating the glob+pickle loop).

Also landed, addressing the tanh-squash note below: ``BCPolicy``/``BC`` gained a
``tanh_squash: bool = True`` flag (default preserves existing behavior).
``False`` builds ``UnsquashedGaussianActor`` (``rl_garden/networks/actor_critic.py``) instead of ``SquashedGaussianActor`` -- motivated by a
concrete correctness argument, not just matching HIL-SERL's
``tanh_squash_distribution=False`` default: tanh-squashed
``evaluate_action_log_prob`` on expert actions sitting at exactly the action
bounds (which real Franka demos produce on a near-binary gripper dimension)
is numerically degenerate, since the tanh Jacobian correction diverges at
the boundary. ``RealWorldBCArgs``/``BCArgs`` (the sim/offline entrypoint too)
both expose it.

.. _en-hil-serl-roadmap-3-reward-classifier-training-script--data-collection-convention----landed:

3. Reward classifier training script + data-collection convention -- LANDED
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Originally, the original classifier module only had the model
class (``SuccessClassifier``) and the inference-side loader
(``load_classifier_fn``) -- enough for ``RewardClassifierWrapper`` to consume an
already-trained checkpoint, but nothing to produce one.

This is now built, and the whole reward-model surface moved to
``rl_garden/models/reward/`` (a pre-existing, previously-unimported directory
of offline HDF5-labeled classifiers -- ``classifiers/{base,color,alignment,hsv}``
-- reorganized so the directory's semantics genuinely match "reward model",
not just "classifier"; future kinds like VLM- or rule-based reward models are
expected to land as further siblings). ``success/`` is the new sibling for the
online real-robot success classifier:

-  ``rl_garden/models/reward/success/model.py`` -- ``SuccessClassifier``/
   ``load_classifier_fn``, moved here unchanged from the original classifier module.
-  ``rl_garden/models/reward/success/data.py`` -- ``SuccessClassifierDataset``,
   reading ``{"obs": obs}`` pickle lists split by success/failure glob pattern
   (mirrors HIL-SERL's ``classifier_data/*_{success,failure}_*.pkl`` convention).
-  ``rl_garden/models/reward/success/train.py`` -- standalone argparse CLI (not
   routed through ``BaseAlgorithmRegistry``; supervised classifier training
   doesn't fit the RL Args/env_backend shape), mirroring HIL-SERL's own
   ``examples/train_reward_classifier.py``: balanced success/failure batch
   sampling, ``binary_cross_entropy_with_logits``, ``RandomShiftsAug``
   (``rl_garden/encoders/augment.py``, reused as-is) for augmentation, no
   train/val split, fixed epoch count.
-  ``real_world_rl/reward/collect_data.py`` delegates to
   ``real_world_rl/reward/collect_real_robot.py``, using station cameras and
   ``s``/``f`` events while preserving success/failure pickle files.

While fixing ``classifiers/hsv/__init__.py``'s pre-existing broken import
(``rl_garden.reward_models...`` -> ``rl_garden.models.reward...``, which made
the whole ``classifiers/hsv`` submodule unimportable), two more latent
import-time bugs surfaced in the same pre-existing ``classifiers/base/`` code
and were fixed as part of making the directory importable at all:
``metrics.py`` eagerly imported ``sklearn`` (not a declared dependency anywhere
in ``pyproject.toml``) and ``transforms.py`` eagerly imported ``torchvision``
(also undeclared) -- both moved to lazy, function-local imports.
``classifiers/hsv/generate_labels.py`` also had two leading module docstrings,
which is a ``SyntaxError`` for the ``from __future__ import annotations`` line
immediately after them; reduced to one.

.. _en-hil-serl-roadmap-4-dual-arm-hybrid-gripper-sac_hybrid_dual:

4. Dual-arm hybrid gripper (``sac_hybrid_dual``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


``RLPDHybrid`` now has a ``3 ** num_grippers`` joint discrete head.
``real_world_rl/envs/real_robot_bimanual/`` supports two arms; two grippers
therefore use nine joint choices. The removed legacy backend was single-arm.

.. _en-hil-serl-roadmap-5-franka-bridge-server-endpoints-set_load-close_gripper_slow-move_gripper-update_param:

5. Franka bridge server endpoints: ``/set_load``, ``/close_gripper_slow``, ``/move_gripper``, ``/update_param``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Comparing ``3rd_party/hil-serl/serl_robot_infra/robot_servers/franka_server.py``
against the base SERL version already ported into
``real_world_rl/legacy/controller/real/franka_bridge.py`` found these HIL-SERL routes
that weren't carried over:

-  ``/set_load`` (POST) -- calls the ROS service ``/franka_control/set_load``
   with ``mass``/``F_x_center_load``/``load_inertia``, presumably for updating the
   arm's dynamics model after a tool/gripper change. Requires a new ROS
   service dependency (``franka_msgs.srv.SetLoad``) not currently wired up.
-  ``/close_gripper_slow`` (POST) -- a slower gripper-close variant
   (Robotiq-specific, see below).
-  ``/move_gripper`` (POST, ``franka_server.py:339-345``) -- continuous gripper
   position control, backed by ``move()`` on both ``franka_gripper_server.py``
   and ``robotiq_gripper_server.py``. ``rl-garden``'s gripper server only exposes
   binary open/close, so no policy action space depending on a continuous
   gripper width can be driven through this bridge today.
-  ``/update_param`` (POST, ``franka_server.py:223-225,378-381``) -- runtime
   dynamic-reconfigure of the cartesian impedance controller's
   stiffness/damping via ``ReconfClient``. No equivalent in ``franka_bridge.py``
   -- impedance parameters can't be tuned at runtime over HTTP.

Not a gap: the many fine-grained getter routes HIL-SERL exposes (``/getpos``,
``/getvel``, ``/getforce``, ``/getq``, ``/getdq``, ``/getjacobian``, etc.) are
deliberately consolidated into ``franka_bridge.py``'s single ``/getstate``
response instead of being ported 1:1 -- see that file's own docstring, which
scopes it to only what the old bridge client actually calls. ``/getstate``'s
response is missing ``q``/``dq``/``jacobian`` specifically (nothing downstream
currently reads them), the removed bridge client originally documented these fields inaccurately.

.. _en-hil-serl-roadmap-6-robotiq-gripper-support:

6. Robotiq gripper support
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

HIL-SERL's ``robotiq_gripper_server.py`` differs from the Franka-hand gripper
server already ported (inverted ``gripper_pos`` reporting, a partial-open
default position instead of fully-open, a lower default close force, plus
the ``close_slow()`` method backing endpoint 5 above). Out of scope: v1's
Franka bridge (``real_world_rl/legacy/controller/real/gripper_server.py``) only
supports the Franka hand, matching the SERL v1 design's original scoping
decision (Robotiq flagged there as "a future addition if needed").

.. _en-hil-serl-roadmap-7-onlinedemo-buffer-split--periodic-disk-snapshot-crash-recovery----landed:

7. Online/demo buffer split + periodic disk snapshot (crash recovery) -- LANDED
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``3rd_party/hil-serl/examples/train_rlpd.py:140-233`` tracks two separate
buffers on the actor side: every transition goes into ``data_store`` (all
online transitions), and only the transitions collected while
``info["intervene_action"]`` is present also go into ``intvn_data_store`` (the
"demo" buffer -- a continuously growing set of human-corrected transitions,
not a static pre-collected dataset). Both are in-memory and are what the
learner actually samples from at train time. Separately, every
``config.buffer_period`` steps, the transitions accumulated since the last
period are pickled to ``checkpoint_path/buffer/transitions_{step}.pkl`` and
``checkpoint_path/demo_buffer/transitions_{step}.pkl`` -- a crash-recovery
snapshot, reloaded by globbing both directories on restart
(``train_rlpd.py:457-490``), not the live sampling source.

Both pieces are now built, reusing existing machinery rather than adding a
third mixing mechanism:

-  **Demo buffer**: ``rl_garden/buffers/demo_intervention.py``'s
   ``DemoInterventionMixin(PriorDataReplayMixin)`` adds ``init_demo_buffer()``/
   ``add_demo_transition()``, populating the *same* ``offline_replay_buffer``/
   ``offline_data_ratio`` slot ``PriorDataReplayMixin`` already uses for RLPD's
   static ``--offline_dataset`` prior dataset -- just grown incrementally
   instead of loaded once. ``_sample_train_batch``/``_concat_replay_samples``/
   ``_shuffle_batch`` are all inherited unchanged. **Known limitation**:
   ``--offline_dataset`` and HIL-SERL's live demo buffer share this one
   slot, so this round doesn't support using both at once on the same
   ``RLPDHybrid`` instance -- see "Item 7 correction" above: this now raises
   instead of silently discarding the loaded offline dataset.
   ``RLPDHybrid(DemoInterventionMixin, RLPD)`` in
   ``rl_garden/algorithms/rlpd_hybrid.py``.

-  **Intervention tagging**: both ordinary and FWBW actor paths retain executed
   and proposed actions, human masks and per-direction trajectory boundaries.
   The shared training projection routes human rows into online and demo replay.

-  **Routing + snapshot**: ``HilSerlLearnerLoop``
   (``real_world_rl/loops/hil_serl.py``) overrides
   ``_on_transition`` to route the way HIL-SERL's own actor loop does
   (``train_rlpd.py:178-193``, corrected -- see below): *every* transition
   (popped ``intervened`` flag) goes into ``agent.replay_buffer``, and intervened
   ones are *additionally* copied into ``agent.add_demo_transition`` -- the demo
   buffer is a duplicated subset of the online buffer, not a disjoint
   partition. Pickles the accumulated-since-last-snapshot transitions
   (pre-device-transfer, CPU tensors, for portability) to
   ``<checkpoint_dir>/buffer/transitions_{n}.pkl`` (every transition) /
   ``<checkpoint_dir>/demo_buffer/transitions_{n}.pkl`` (intervened only) every
   ``--buffer_period`` received transitions, glob-reloading both directories on
   startup (received-count is tallied off the "buffer" reload only, since
   "demo_buffer" entries are duplicates of ones already in "buffer" -- reloading
   both without this would double-count intervened transitions). This was the migration-era persistence path. Current bundles restore the complete
   replay state and journal cursor; seed demos and snapshots are not reloaded over a
   restored bundle. See :doc:`Real-world experiments <real-world-experiments>`.

   **Correction**: the initial landing routed exclusively (intervened ->
   demo buffer *only*, non-intervened -> online buffer *only*), which is not
   what HIL-SERL does -- its actor inserts every transition into ``data_store``
   unconditionally and *additionally* inserts intervened ones into
   ``intvn_data_store`` (``train_rlpd.py:178-193``). Under the old exclusive
   routing, ``rl-garden``'s online buffer never contained any human-corrected
   data at all, changing the online-half sampling distribution relative to
   HIL-SERL (where intervened transitions are present in both the online and
   demo halves of the mix). Fixed to match, scoped entirely to
   ``HilSerlLearnerLoop`` (no change to ``DemoInterventionMixin``, ``ActorLoop``, or
   base ``LearnerLoop``).

-  New ``HilSerlArgs`` fields: ``demo_buffer_size``, ``demo_data_ratio``,
   ``buffer_period`` (now declared on ``HilEnvArgs``, which ``HilSerlArgs``,
   ``ResidualHilSerlArgs`` and ``SilriArgs`` share).

**Update (shared loop seams)**: the tagging and routing above are now
expressed through overridable hooks, with the ``hil_serl`` defaults unchanged.
``ActorLoop.run()`` calls ``_act``, ``_build_transition``, ``_push``,
``_on_episode_end``, ``_on_params_refreshed`` and ``_on_run_end``;
``HilSerlActorLoop`` also overrides transition construction, policy refresh and
episode statistics hooks. The intervention
wrapper now writes ``info["intervened"]`` (always) next to ``intervene_action``
(only while intervening); ``HilSerlActorLoop`` keeps tagging by
``"intervene_action" in info``. ``HilSerlLearnerLoop._buffer_kwargs(transition) -> (core, extra)`` decides which fields reach ``replay_buffer.add``; the default
drops ``intervened`` and ``actor_id`` as before, and the residual and SiLRI loops
override it to keep ``intervened``, ``policy_action`` and their own columns for
``InterventionReplayBuffer``.

.. _en-hil-serl-roadmap-12-gripper_pos-normalization-bug--other-robot_servers-deviations:

12. ``gripper_pos`` normalization bug + other ``robot_servers`` deviations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``real_world_rl/legacy/controller/real/gripper_server.py`` was ported from HIL-SERL's
``franka_gripper_server.py`` but diverges from it in ways not previously
caught:

-  **Gripper normalization is fixed**: ``FrankaGripperServer._on_joint_state``
   computes ``float(sum(msg.position)) / 0.08``, giving normalized width
   rather than metres.
-  **No open/close idempotence guard**: HIL-SERL tracks
   ``self.binary_gripper_pose`` and short-circuits a repeated ``/open_gripper``
   or ``/close_gripper`` call if the gripper is already in that state
   (``franka_gripper_server.py:21-43``). ``gripper_server.py`` has no such guard
   -- every call unconditionally republishes a new
   ``MoveActionGoal``/``GraspActionGoal``, sending redundant grasp commands every
   control step a policy holds the gripper closed/open.
-  ``load_gripper`` **roslaunch arg not passed explicitly**: HIL-SERL's
   ``franka_server.py`` passes ``load_gripper:=true/false`` to ``impedance.launch``/
   ``joint.launch`` based on gripper type. ``franka_bridge.py`` omits this arg
   entirely -- currently benign since ``3rd_party/serl_franka_controllers``'s
   launch files default it to ``true`` (the correct value, since ``rl-garden``
   only supports the Franka hand today), but implicit rather than explicit.

.. _en-hil-serl-roadmap-13-safety-box-only-clamps-position-not-orientation----landed-correction-already-was:

13. Safety box only clamps position, not orientation -- LANDED (correction: already was)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Correction**: this item's original text was stale/inaccurate by the time
of a later audit -- orientation clamping already existed.
The historical env config had
``safety_box_rpy_low``/``safety_box_rpy_high`` (``Optional``, both-or-neither, off
by default). Its orientation helper converted to Euler ``"xyz"``, clips the first angle by absolute value with sign
preserved (the same wraparound special-case HIL-SERL's
``clip_safety_box()`` uses), clips the rest directly, and converts back --
applied in ``step()`` whenever ``safety_box_rpy_low`` is set. Nothing to build;
noted here only so this doesn't get re-flagged as a gap. The one real
difference from HIL-SERL: orientation clamping is opt-in (``None`` by
default) rather than always-on.

.. _en-hil-serl-roadmap-14-no-quaternion-to-eulerrotvec-conversion-on-the-observation-side----landed-correction-already-was:

14. No quaternion-to-Euler/rotvec conversion on the observation side -- LANDED (correction: already was)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Correction**: also already existed. ``RotvecObsWrapper`` converts
``obs["state"]``'s quaternion slice to a 3D rotation vector (not Euler -- a
deliberate, already-documented deviation: "HIL-SERL converts to Euler via
``Quat2EulerWrapper`` instead, but rotvec avoids Euler's gimbal-lock/wraparound
special-casing and stays consistent with this repo's own action convention"),
wired into ``training/env_building.py::compose_hil_env`` (formerly ``hil_serl.py::_build_env``) behind ``--convert_obs_rotation`` (off by
default). Nothing to build; noted here only so this doesn't get re-flagged.
Moved from ``rl_garden/envs/wrappers/rotvec_obs.py`` to
``real_world_rl/envs/wrappers/rotvec_obs.py`` during a later placement
audit (it hardcodes the legacy env's exact 20-dim state layout, so it never
belonged in the general ``rl_garden/`` wrappers package -- its own test was
already correctly in ``real_world/tests/``).

.. _en-hil-serl-roadmap-15-missing-franka_env-wrapper-variants-humanclassifierwrapper-multistagebinaryrewardclassifierwrapper-grippercloseenv----landed:

15. Missing ``franka_env`` wrapper variants: ``HumanClassifierWrapper``, ``MultiStageBinaryRewardClassifierWrapper``, ``GripperCloseEnv`` -- LANDED
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Built, batched-torch, following ``RewardClassifierWrapper``'s convention:
``HumanClassifierWrapper`` (``real_world_rl/envs/wrappers/human_classifier.py`` -- lives in ``real_world/``, not ``rl_garden/``, since
blocking ``input()`` is meaningless outside a live teleop session);
``MultiStageBinaryRewardClassifierWrapper`` (``rl_garden/envs/wrappers/multi_stage_classifier.py`` -- shape-agnostic, no action-space assumptions,
so it lives in the general ``rl_garden/`` wrappers package; each stage fires
reward exactly once, ``terminated`` flips once all stages received);
``GripperCloseEnv`` (``real_world_rl/envs/wrappers/gripper_close.py`` --
7D->6D action space, zero-pads back before forwarding, truncates
``info["intervene_action"]`` too. Originally landed under ``rl_garden/``, moved
here: the 7D/last-dim-is-gripper shape it asserts is the legacy env's own
action convention, not a generic one -- no ManiSkill/RoboTwin sim task uses
it, unlike ``MultiStageBinaryRewardClassifierWrapper``'s genuine
shape-agnosticism).

Original text below, left for context:

``RewardClassifierWrapper`` (``rl_garden/envs/wrappers/reward_classifier.py``,
item 3 above) only covers HIL-SERL's simplest case,
``MultiCameraBinaryRewardClassifierWrapper`` (single-stage, model-driven
success). Two other wrapper variants HIL-SERL provides
(``3rd_party/hil-serl/serl_robot_infra/franka_env/envs/wrappers.py``) have no
``rl-garden`` equivalent:

-  ``HumanClassifierWrapper`` (``wrappers.py:15-34``) -- blocks on ``input()`` at
   episode end for a human to type success/failure, an alternative to the
   learned classifier for tasks where training one isn't worth it yet.
-  ``MultiStageBinaryRewardClassifierWrapper`` (``wrappers.py:69-100``) --
   sequential multi-stage success tracking (per-stage ``received`` bookkeeping,
   summed reward, episode ends only once every stage is satisfied) -- needed
   for tasks with multiple sequential sub-goals.

Also missing: ``GripperCloseEnv`` (``wrappers.py:180-204``), an ``ActionWrapper``
that truncates the action space to 6D (drops the gripper dimension,
zero-padding it back) for gripper-always-closed tasks -- no equivalent
action wrapper exists in ``rl_garden/envs/wrappers/``.

.. _en-hil-serl-roadmap-16-pico-and-spacemouse-intervention-detection-are-structurally-different-informational:

16. Pico and SpaceMouse intervention detection are structurally different (informational)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``real_world_rl/legacy/teleop/spacemouse/spacemouse_teleop_wrapper.py``'s
``SpaceMouseTeleOpWrapper`` closely mirrors HIL-SERL's own
``SpacemouseIntervention`` (``3rd_party/hil-serl/serl_robot_infra/franka_env/envs/wrappers.py:207-252``): always-on rate control, intervention decided by
a pure magnitude deadzone (``norm(expert_a) > threshold``). ``rl-garden``'s
other teleop device, ``real_world_rl/teleop/pico/bridge.py``'s
``EETwistTeleOpWrapper`` ("pico"), is *bind-gated* instead -- it only reports
a nonzero (and therefore "intervened") twist while a physical bind button is
held, computing a delta-since-bind, and falls back to a zero/not-intervened
sample whenever no fresh device packet has arrived. Both converge on the
same ``TeleOpSample.intervened`` contract consumed by
``TeleopInterventionWrapper``, but the underlying activation semantics differ:
physically moving a Pico controller without pressing its bind button never
triggers intervention, whereas any SpaceMouse motion above the deadzone
always will. Not a bug -- just worth knowing which device gives which
behavior when choosing ``--teleop_device``.

**Current update**: ``--teleop-device spacemouse`` uses the in-tree HID decoder
(``hidapi==0.14.0.post4``), not pyspacemouse. Default ``--spacemouse-mode tracker``
uses button 0 for takeover/hand-back and button 1 for the sticky gripper.
``--spacemouse-mode serl`` uses motion/button takeover, release to return control
and held-button gripper commands. Outcome labels come from the event device.
Pico remains bind-gated. See :doc:`Teleop <teleop>`.

.. _en-hil-serl-roadmap-17-no-real-camera-capture-integration:

17. No real camera capture integration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The ``real_robot`` and ``real_robot_bimanual`` backends have device-layer camera
integration, including RealSense and ZED backends in
``real_world_rl/devices/backends/cameras/``. HIL runs can receive these images locally
or through the daemon.

.. _en-hil-serl-roadmap-18-no-real-world-rollout-video-recording----landed:

18. No real-world rollout video recording -- LANDED
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Built: ``real_world_rl/envs/wrappers/video_recorder.py:: VideoRecorderWrapper``, combining both upstream ideas (accumulates obs-stream
frames like ``VideoWrapper``, writes to disk on episode end like
``VideoRecorder``). Pipes raw frames to an ``ffmpeg`` subprocess -- the same
convention ``rl_garden/envs/robotwin/adapter.py``'s eval-video logging already
uses -- rather than adding an ``imageio``/``tensorflow`` dependency. Wired into
``_run_actor`` of ``hil_serl.py``, ``residual_hil_serl.py`` and ``silri.py`` (actor role only) behind a new ``record_video_dir``
``HilEnvArgs`` field (shared by every HIL-style Args).

Original text below, left for context:

HIL-SERL's ``VideoRecorder``/``VideoWrapper``
(``3rd_party/hil-serl/serl_launcher/serl_launcher/wrappers/{video_recorder,video_wrapper}.py``) record rollout video to disk for debugging. ``rl-garden``
has no equivalent anywhere in the real-world path (the only video-writing
code in the repo, ``rl_garden/envs/robotwin/adapter.py``, is RoboTwin
simulation eval logging, unrelated). There is currently no way to record
video of a real-robot ``hil_serl``/``serl`` rollout for after-the-fact
debugging.

.. _en-hil-serl-roadmap-19-replay-buffer-scalability-gaps-no-image-frame-dedup-no-bounded-actor-transport-queue----landed:

19. Replay-buffer scalability gaps: no image-frame dedup, no bounded actor-transport queue -- LANDED
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Both pieces built:

-  ``pack_obs_and_next_obs`` **frame dedup**: new
   ``rl_garden/buffers/memory_efficient_buffer.py:: MemoryEfficientReplayBuffer`` (subclasses ``ReplayBuffer``). Stores
   one frame per timestep per designated image key; reconstructs the
   ``(T, H, W, C)`` stacked window at sample time via a torch-native windowed
   gather (no NumPy ``sliding_window_view``, staying GPU-resident). Edge-
   replicates at episode boundaries (matching ``ImageFrameStackWrapper``'s own
   ``reset()`` behavior) rather than reject-sampling them; a per-slot episode-id
   check guards the one genuine hazard (a stale, still-resident position
   whose temporal neighbor has since been overwritten by a later episode --
   verified by test to be reachable and correctly rejected, not just a
   theoretical concern). Requires ``ImageFrameStackWrapper`` upstream in the env
   wrapper stack; ``add()`` asserts this explicitly rather than silently
   mis-shaping storage. Opt-in via ``RLPDHybrid(memory_efficient_buffer=True, ...)`` / ``HilSerlArgs.memory_efficient_buffer``.
-  **Durable actor outbox**: ``ActorSyncClient`` now commits uploads to SQLite and
   retries without dropping rows. ``max_queue_size`` is only a compatibility argument.
   Capacity defaults to 100,000 records or 1 GiB; exhaustion stops collection. See
   :doc:`Real-world experiments <real-world-experiments>`.

.. _en-hil-serl-roadmap-20-temporal-observation-stacking-not-wired-into-the-real-world-path----landed:

20. Temporal observation stacking not wired into the real-world path -- LANDED
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``ImageFrameStackWrapper`` (``rl_garden/envs/wrappers/frame_stack.py``) gained a
generic (non-ManiSkill) path: an explicit ``image_keys`` constructor param
(the legacy env's camera keys are user-configured, not necessarily
``rgb*``/``depth*``-prefixed) that derives the stacked ``single_observation_space``
directly from the space definition -- deliberately never calls ``env.reset()``
to seed it, since a construction-time side effect would physically move the
robot. Wired into ``training/env_building.py::compose_hil_env`` (formerly ``hil_serl.py::_build_env``), immediately after
``make_training_envs(...)``, gated on the existing ``obs.frame_stack``
field (the observation group inherited from ``RLPDHybridArgs`` -- no duplicate field added).
The ManiSkill (``image_keys=None``) path is untouched.

Original text below, left for context:

HIL-SERL's ``ChunkingWrapper``
(``3rd_party/hil-serl/serl_launcher/serl_launcher/wrappers/chunking.py``)
stacks *all* observation keys (state + images) into a temporal window and is
environment-agnostic -- applied uniformly to sim and real robot envs alike,
plus supports receding-horizon action-chunk execution. ``rl-garden``'s closest
analog, ``ImageFrameStackWrapper``
(``rl_garden/envs/wrappers/frame_stack.py``), only stacks image keys (state
stays single-frame) and has no action-chunk concept, and more importantly is
only wired into the ManiSkill simulation backend
(``rl_garden/envs/maniskill/env.py``) -- ``hil_serl``'s ``_build_env``
(``real_world_rl/training/entrypoints/hil_serl.py``) never applies it. Real Franka
rollouts currently get no temporal observation stacking at all.

.. _en-hil-serl-roadmap-landed-since-the-initial-migration:

Landed since the initial migration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Not HIL-SERL items, but they change what "not migrated" means above; all are
real-world loops only (no simulator ``learn()`` path):

-  **Teleop base class**: ``TeleopSource``/``TeleopReading`` and a name registry
   (``teleop/``), device-layer SpaceMouse, and a generic ``InterventionWrapper``
   (item 16 update; ``docs/teleop.md``).
-  **Shared HIL env builder**: ``training/env_building.py::build_hil_env(args, *, role, enable_teleop)``; the learner passes ``enable_teleop=False``, so no
   teleop device is opened on the GPU box. ``hil_serl._build_env`` remains as a
   thin alias of ``compose_hil_env``.
-  **Residual RL with human intervention** (``residual_hil_serl``): a bounded
   residual over a frozen base policy (rl-garden checkpoint, ACT or zero), both
   ``ResidualRLPD`` and ``ResidualRLPDHybrid``, with the human overriding the total
   action. This is a human-in-the-loop method HIL-SERL itself does not have
   (``docs/residual.md``).
-  **HIL-RL / SiLRI** (``silri``): learning from suboptimal interventions with a
   state-wise Lagrangian (:doc:`SiLRI <silri>`). HG-DAgger remains outside the
   implemented methods. Leader-arm teleop and dual-arm grippers are implemented
   (:doc:`Teleop <teleop>`, :doc:`Bimanual <bimanual>`).

.. _en-hil-serl-roadmap-behavioral-divergences-from-hil-serls-real-robot-defaults:

Behavioral Divergences from HIL-SERL's Real-Robot Defaults
--------------------------------------------------------------------------------

Current defaults below distinguish ``HilSerlArgs`` from generic rl-garden defaults.
The historical HIL-SERL comparisons refer to its vendored real-robot recipes.

-  **Entropy backup is configurable**: ``HilSerlArgs.backup_entropy=False``
   matches HIL-SERL. ``--backup-entropy`` enables it; the generic SAC default
   remains ``True``.
-  **Actor-side LayerNorm defaults off**: ``RLPDHybridArgs`` overrides
   ``critic_use_layer_norm=True`` but not ``actor_use_layer_norm`` (inherits
   ``False``, ``rl_garden/training/online/args.py``). HIL-SERL's
   ``make_sac_pixel_agent_hybrid_single_arm`` sets ``use_layer_norm=True`` on
   both critic and policy kwargs (``launcher.py:120-130``).
-  **Image augmentation is enabled by default**: ``HilSerlArgs.encoder`` sets
   ``image_augmentation="random_shift"``. Override with
   ``--encoder.image-augmentation``; the old top-level flag is not the current CLI.
-  **ResNet backbone defaults unfrozen, from-scratch**: HIL-SERL's real-robot
   configs (all using ``encoder_type="resnet-pretrained"``) unconditionally
   freeze the backbone (``stop_gradient`` in ``resnet_v1.py``'s ``pre_pooling``),
   training only the pooling head. ``rl-garden``'s ``ResNetEncoder`` only freezes
   when the caller passes both ``pretrained_weights`` *and*
   ``freeze_resnet_encoder``/``freeze_resnet_backbone=True``; ``EncoderConfig``
   leaves pretrained weights and freezing unset. If a ResNet is selected without
   overriding these fields, its whole backbone trains from scratch; the default
   HIL encoder itself is ``plain_conv``.
-  **Pooling matches the HIL-SERL setting**: ``HilSerlArgs.encoder`` sets
   ``pooling_method="spatial_learned_embeddings"``. This applies when selecting
   a ResNet encoder; the default encoder is ``plain_conv``. Generic
   ``ResNetEncoder`` still defaults to ``spatial_softmax``.
-  **REDQ ensemble defaults reflect the RLPD paper, not HIL-SERL's real
   practice**: ``RLPDHybridArgs`` defaults to ``n_critics=10, critic_subsample_size=2`` (matching ``RLPD``'s own docstring). HIL-SERL's
   actual real-robot launcher calls for every agent type (RLPD, hybrid
   single, hybrid dual) hardcode ``critic_ensemble_size=2, critic_subsample_size=None`` (``launcher.py:86-87,136-137,186-187``) -- no
   REDQ subsampling was used on real robots in practice.
-  **Discrete (gripper) critic always stop-gradients the shared encoder**:
   ``RLPDHybrid._train_discrete_critic`` calls ``extract_critic_features(..., stop_gradient=True)`` unconditionally
   (``rl_garden/algorithms/rlpd_hybrid.py``). HIL-SERL's ``grasp_critic_loss_fn``
   backpropagates into the grasp critic's own pooling head
   (``sac_hybrid_single.py:233-289``, ``grad_params=params``) -- only the
   pretrained backbone convolutions are frozen, not the whole encoder path
   from the discrete critic's perspective.
-  **FiLM conditioning is unused on both sides** (not a gap): HIL-SERL's
   ``FilmConditioning`` layer is only reachable through ``-film`` ResNet config
   variants that none of its real-robot agent constructors ever select.
   ``rl_garden/encoders/film.py`` is likewise an explicit, documented stub not
   wired into any algorithm. Noted here so it isn't mistakenly re-flagged as
   a missing feature later.
