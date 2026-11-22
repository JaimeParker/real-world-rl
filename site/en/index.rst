real-world-rl tutorials
================================================================================

.. container:: expnote-home-intro

   English chapters will follow once the Chinese tutorials are final. See :doc:`the Chinese home </index>`.

.. rubric:: Getting started

.. grid:: 1 1 2 2
   :gutter: 3

   .. grid-item-card:: Install and First Check
      :link: tutorials/install
      :link-type: doc
      :shadow: none

      English version pending.

.. rubric:: Station and teleoperated recording

.. grid:: 1 1 2 2
   :gutter: 3

   .. grid-item-card:: Build the Station
      :link: tutorials/station
      :link-type: doc
      :shadow: none

      English version pending.

   .. grid-item-card:: Choose and Configure the Franka Controller
      :link: tutorials/controller
      :link-type: doc
      :shadow: none

      English version pending.

   .. grid-item-card:: Teleoperate with a SpaceMouse
      :link: tutorials/spacemouse
      :link-type: doc
      :shadow: none

      English version pending.

   .. grid-item-card:: Teleoperate with GELLO
      :link: tutorials/gello
      :link-type: doc
      :shadow: none

      English version pending.

   .. grid-item-card:: Record and Label Demonstrations
      :link: tutorials/record
      :link-type: doc
      :shadow: none

      English version pending.

   .. grid-item-card:: Inspect a Recording
      :link: tutorials/inspect
      :link-type: doc
      :shadow: none

      English version pending.

.. rubric:: Behaviour cloning (BC) from demonstrations

.. grid:: 1 1 2 2
   :gutter: 3

   .. grid-item-card:: Prepare BC Training Data
      :link: tutorials/bc-data
      :link-type: doc
      :shadow: none

      English version pending.

   .. grid-item-card:: Train a First BC Policy
      :link: tutorials/bc-train
      :link-type: doc
      :shadow: none

      English version pending.

   .. grid-item-card:: Check a BC Checkpoint Offline
      :link: tutorials/bc-check
      :link-type: doc
      :shadow: none

      English version pending.

.. rubric:: Deployment and inference

.. grid:: 1 1 2 2
   :gutter: 3

   .. grid-item-card:: Deployment and Inference
      :link: tutorials/deploy
      :link-type: doc
      :shadow: none

      A: deploy a BC checkpoint trained here. B: deploy an existing openpi / RLinf policy. English versions pending.

.. rubric:: Real-robot reinforcement learning (HIL-SERL)

.. grid:: 1 1 2 2
   :gutter: 3

   .. grid-item-card:: Understand an Online RL Run
      :link: tutorials/rl-overview
      :link-type: doc
      :shadow: none

      English version pending.

   .. grid-item-card:: Prepare a Trainable Task
      :link: tutorials/rl-task
      :link-type: doc
      :shadow: none

      English version pending.

   .. grid-item-card:: Start HIL-SERL Training
      :link: tutorials/rl-train
      :link-type: doc
      :shadow: none

      English version pending.

   .. grid-item-card:: Correct the Policy During Training
      :link: tutorials/rl-intervene
      :link-type: doc
      :shadow: none

      English version pending.

   .. grid-item-card:: Evaluate and Resume Training
      :link: tutorials/rl-resume
      :link-type: doc
      :shadow: none

      English version pending.

.. toctree::
   :maxdepth: 2
   :caption: Getting started
   :hidden:

   tutorials/install

.. toctree::
   :maxdepth: 2
   :caption: Station and teleoperated recording
   :hidden:

   tutorials/station
   tutorials/controller
   tutorials/spacemouse
   tutorials/gello
   tutorials/record
   tutorials/inspect

.. toctree::
   :maxdepth: 2
   :caption: Behaviour cloning (BC) from demonstrations
   :hidden:

   tutorials/bc-data
   tutorials/bc-train
   tutorials/bc-check

.. toctree::
   :maxdepth: 2
   :caption: Deployment and inference
   :hidden:

   tutorials/deploy
   tutorials/deploy-openpi

.. toctree::
   :maxdepth: 2
   :caption: Real-robot reinforcement learning (HIL-SERL)
   :hidden:

   tutorials/rl-overview
   tutorials/rl-task
   tutorials/rl-train
   tutorials/rl-intervene
   tutorials/rl-resume
