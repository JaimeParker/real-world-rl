real-world-rl
=============

.. container:: expnote-home-intro

   Record a demonstration on a real robot, train an ACT policy, and deploy it
   before starting online learning. If you already have a checkpoint, start with deployment.

.. grid:: 1 1 2 2
   :gutter: 3

   .. grid-item-card:: Installation
      :link: install
      :link-type: doc
      :shadow: none

      Choose dependencies for the device host, client, or learner.

   .. grid-item-card:: Teleoperation and data collection
      :link: collect
      :link-type: doc
      :shadow: none

      Record and inspect a demonstration with SpaceMouse or GELLO.

   .. grid-item-card:: Behavior cloning with ACT
      :link: bc
      :link-type: doc
      :shadow: none

      Predict a chunk of actions from a recorded observation.

   .. grid-item-card:: Deployment and inference
      :link: deploy
      :link-type: doc
      :shadow: none

      Run ACT locally or connect to a remote π0.5 policy service.

   .. grid-item-card:: Real-world reinforcement learning
      :link: rl
      :link-type: doc
      :shadow: none

      Follow a correction from the actor to the learner.

.. toctree::
   :maxdepth: 2
   :hidden:

   install
   collect
   bc
   deploy
   openpi
   rl
