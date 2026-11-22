.. _en-act-action-chunking-with-transformers-act:

Action Chunking with Transformers (ACT)
================================================================================

`Chinese entry point <https://github.com/JaimeParker/real-world-rl/blob/main/docs/README.zh-CN.md>`__ · `English <https://github.com/JaimeParker/rl-garden/blob/main/rl_garden/models/act/README.md>`__

This code runs the ACT algorithm from the paper `Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware <https://arxiv.org/pdf/2304.13705>`__, adapted from the `original code <https://github.com/tonyzhaozh/act>`__.

.. _en-act-installation:

Installation
--------------------------------------------------------------------------------

Before starting, we recommend creating a new environment with conda/mamba and installing the dependencies:

.. tutorial-test: skip
.. code-block:: bash

   conda create -n act-ms python=3.10
   conda activate act-ms
   pip install -e ./rl-garden -e '.[learning]' torchvision

.. _en-act-setup:

Setup
--------------------------------------------------------------------------------

For the legacy ManiSkill baselines, read the `imitation learning setup documentation <https://maniskill.readthedocs.io/en/latest/user_guide/learning_from_demos/setup.html>`__ in full. It covers what you need to run imitation learning baselines in ManiSkill, including how to download and preprocess demonstration datasets, how to evaluate and compare policies fairly, and tips for improving performance and avoiding errors.

.. _en-act-training:

Training
--------------------------------------------------------------------------------


The legacy ManiSkill scripts are ``rl-garden/rl_garden/models/act/train.py`` and
``train_rgbd.py``; the baseline commands are in the same directory's ``baselines.sh``.
They depend on ManiSkill and legacy top-level ``act``/``envs`` imports and are not
repository-root entrypoints. For this repository's real-robot demonstrations, use
``python -m rl_garden.models.act.train_trajectories`` below; it needs no simulator.

.. _en-act-training-from-trajectory-pkls:

Training from trajectory pkls
--------------------------------------------------------------------------------

``train_trajectories.py`` trains ACT from lists of transitions saved in pickle format, without a simulator or ManiSkill, for data such as teleoperated demonstrations recorded on a real robot. Run from the repository root:

.. code-block:: bash

   python -m rl_garden.models.act.train_trajectories \
     --demo-paths "demos/*.pkl" \
     --total-iters 50000 --batch-size 256 --num-queries 30 \
     --exp-name act-my-task

-  Data: each pkl stores a chronological list of transition dictionaries containing ``obs``, ``action`` and ``done``, optionally ``next_obs`` and ``truncated``; a leading batch axis of size 1 is supported. ``obs["state"]`` is the state with shape ``(D,)``; each ``rgb_<camera>`` key corresponds to one camera for a visual policy, with images of shape ``(H, W, 3)`` and type uint8, and all cameras must have the same image dimensions. ``--no-use-cameras`` trains on state only. If ``obs`` itself is a tensor, it is also treated as state.
-  Episodes end at ``done`` or ``truncated``; a new episode is also split off when a transition's ``next_obs`` differs from the next transition's ``obs`` (disable this with ``--no-split-on-observation-gaps``). Action chunks do not cross episode endings: entries past the end are padded and excluded from the L1 loss.
-  States and actions are standardized using statistics computed from the data; images are resized to ``--image-size`` and normalized with ImageNet statistics. ``--backbone-weights`` initializes ResNet and can be ``imagenet`` (the default, downloading torchvision's ``IMAGENET1K_V1``), ``none``, or a path to a torchvision state-dict file for offline machines.
-  Output: ``runs/<exp-name>/checkpoints/final.pt`` (optionally a checkpoint every ``--save-freq`` iterations as ``<iter>.pt``; the default ``None`` disables periodic saves), containing ``agent``, ``ema_agent``, ``norm_stats`` and the full ``act_config``. ``ACTBaseActionProvider.from_checkpoint(ckpt_path=...)`` loads the checkpoint with EMA weights by default. No evaluation runs during training.

The ACT trajectory loader does not consume ``human_action_mask`` or the versioned
training-segment metadata used by BC/HIL. Use fully supervised demonstrations and
retain observation-gap splitting; it cannot supply mask-aware partial imitation.

.. _en-act-citation:

Citation
--------------------------------------------------------------------------------

If you use this baseline, please cite the following reference:

.. code-block:: text

   @inproceedings{DBLP:conf/rss/ZhaoKLF23,
     author       = {Tony Z. Zhao and
                     Vikash Kumar and
                     Sergey Levine and
                     Chelsea Finn},
     editor       = {Kostas E. Bekris and
                     Kris Hauser and
                     Sylvia L. Herbert and
                     Jingjin Yu},
     title        = {Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware},
     booktitle    = {Robotics: Science and Systems XIX, Daegu, Republic of Korea, July
                     10-14, 2023},
     year         = {2023},
     url          = {https://doi.org/10.15607/RSS.2023.XIX.016},
     doi          = {10.15607/RSS.2023.XIX.016},
     timestamp    = {Thu, 20 Jul 2023 15:37:49 +0200},
     biburl       = {https://dblp.org/rec/conf/rss/ZhaoKLF23.bib},
     bibsource    = {dblp computer science bibliography, https://dblp.org}
   }

