Installation and initial checks
================================================================================

1. Which setup do you have?
--------------------------------------------------------------------------------

First choose where the programs will run, so you install only the dependencies that machine needs.

An ``extra`` is an optional group of dependencies, specified after ``--extras`` during installation.
The ``daemon`` is the device service that stays on the NUC. Other machines use it to read state and send actions.
The examples below use a Franka Hand, RealSense and SpaceMouse.

Get the source on the chosen machine and enter the repository root. If you already have a checkout, enter that directory.

.. tutorial-test: skip
.. literalinclude:: ../../examples/install.sh
   :language: bash
   :start-after: # start checkout
   :end-before: # end checkout

Choose one of the following four installation commands for this machine.

* **Record, run inference or run the actor directly on the NUC**: run on the NUC, with the SpaceMouse plugged in there too.

  .. tutorial-test: skip
  .. literalinclude:: ../../examples/install.sh
     :language: bash
     :start-after: # start direct
     :end-before: # end direct

* **Use the NUC only as a device server**: run on the NUC. Recording or inference runs on another machine.

  .. tutorial-test: skip
  .. literalinclude:: ../../examples/install.sh
     :language: bash
     :start-after: # start daemon
     :end-before: # end daemon

* **Record or run inference through the NUC from another machine**: run on the client, with the SpaceMouse plugged into the client.

  .. tutorial-test: skip
  .. literalinclude:: ../../examples/install.sh
     :language: bash
     :start-after: # start client
     :end-before: # end client

* **Run only the learner or offline BC on a GPU server**: run on the GPU server.

  .. tutorial-test: skip
  .. literalinclude:: ../../examples/install.sh
     :language: bash
     :start-after: # start learner
     :end-before: # end learner

Install each device SDK on the machine that actually opens the device. The NUC in the first two setups needs ``franky``;
the last two setups do not. The package installed by the ``franky`` extra is named ``franky-control``.
This is the official franky on PyPI. If you later choose the ``serl`` controller, franky and ``serl_controller`` will be rebuilt together from source on the NUC.
See step 4B of :doc:`controller` (install normally here; the build script will uninstall it first).

.. code-block:: text

   Franka + Hand <--> NUC <--- observations / actions ---> remote recording or inference client
   RealSense ------> NUC
                     |
                     +--- samples / new parameters ---> GPU learner
                     +--- observations / policy actions <-> GPU policy server

   For direct recording, the recorder runs on the NUC without going through the daemon.

**What you should see**: installation ends with ``Installation complete`` and a prompt to activate ``.venv``.

**If it doesn't**:

* ``python3.10`` is not found: add ``--python`` to the installation command and specify an installed Python 3.10 or newer.
* Project files are not found: return to the root of the repository you just checked out and rerun the installation command.
* The GPU runs only an openpi policy server: follow that model server's installation procedure. The ``learning`` extra here is for this repository's training programs.

2. Activate the installed environment
--------------------------------------------------------------------------------

Activate the environment you just installed in the current terminal, so subsequent commands use its dependencies.

.. tutorial-test: skip
.. literalinclude:: ../../examples/install.sh
   :language: bash
   :start-after: # start activate
   :end-before: # end activate

Without ``--extras`` or ``--daemon-only``, ``scripts/install.sh`` defaults to
``learning,franka-gello,shell``. The ``shell`` extra installs the dependencies for the interactive
``rwrl shell`` terminal.

The script uses ``python3.10`` to create ``.venv`` by default. When you select ``learning``, it also installs
the rl-garden submodule and the ``test`` extra. ``--daemon-only`` is shorthand for ``franky,grpc``;
the device-server command in this chapter also adds ``realsense`` for the camera.

You can also pass the following options to the installation script. Run ``bash scripts/install.sh --help`` for the full list.

.. list-table:: Installation options
   :header-rows: 1

   * - Option
     - Purpose
   * - ``--venv PATH``
     - Create or reuse the specified venv. The default is ``.venv``; activate this path afterwards.
   * - ``--torch-index URL``
     - Install torch from the specified index first. Requires ``learning`` as well.
   * - ``--station YAML``
     - Run doctor on the specified real station at the end. Without it, run a mock check.
   * - ``--no-doctor``
     - Skip the final doctor run. Successful installation does not mean device checks passed.

For manual installation, the following dependency steps are equivalent to “record directly on the NUC” in this chapter.
If you already have a project venv or a non-base conda environment, skip the two lines that create and activate the venv.
Clear any inherited ``PYTHONPATH`` first to avoid importing another checkout. For other machines, choose extras as in step 1;
a device-only server does not install ``rl-garden`` or ``learning``.

.. tutorial-test: skip
.. literalinclude:: ../../examples/install.sh
   :language: bash
   :start-after: # start manual
   :end-before: # end manual

After manual installation, still run the checks in step 4.

**What you should see**: activation usually prints nothing. The terminal prompt may show ``(.venv)``.
The import commands below will confirm the environment. When running across machines, save the commit IDs from the two Git commands above
so all machines use the same version.

**If it doesn't**:

* The submodule directory is empty: run ``git submodule update --init`` from the repository root, then rerun the installation command.
* A new terminal cannot find ``rwrl``: run the activation command above again.

3. Add dependencies for your devices
--------------------------------------------------------------------------------

Add the extra for each device you actually use to your chosen installation command. Then check its import on the machine that reads that device.

.. list-table:: Device dependencies
   :header-rows: 1
   :widths: 22 28 50

   * - Device
     - Extra to add
     - One check command after installation
   * - Franka / Franka Hand
     - ``franky``
     - .. literalinclude:: ../../examples/install.sh
          :language: bash
          :start-after: # start check-franky
          :end-before: # end check-franky
   * - RealSense
     - ``realsense``
     - .. literalinclude:: ../../examples/install.sh
          :language: bash
          :start-after: # start check-realsense
          :end-before: # end check-realsense
   * - ZED UVC color
     - ``zed-uvc`` (OpenCV, no depth)
     - .. literalinclude:: ../../examples/install.sh
          :language: bash
          :start-after: # start check-zed-uvc
          :end-before: # end check-zed-uvc
   * - ZED SDK
     - No extra; install the vendor's ``pyzed`` bindings
     - .. literalinclude:: ../../examples/install.sh
          :language: bash
          :start-after: # start check-zed
          :end-before: # end check-zed
   * - SpaceMouse
     - ``spacemouse`` (``hidapi==0.14.0.post4``)
     - .. literalinclude:: ../../examples/install.sh
          :language: bash
          :start-after: # start check-spacemouse
          :end-before: # end check-spacemouse
   * - GELLO
     - ``gello``
     - .. literalinclude:: ../../examples/install.sh
          :language: bash
          :start-after: # start check-gello
          :end-before: # end check-gello
   * - GELLO keyboard / pedal
     - ``gello-events`` (``evdev``)
     - .. literalinclude:: ../../examples/install.sh
          :language: bash
          :start-after: # start check-events
          :end-before: # end check-events
   * - Pico VR
     - ``pico`` (``pyzmq``)
     - .. literalinclude:: ../../examples/install.sh
          :language: bash
          :start-after: # start check-pico
          :end-before: # end check-pico
   * - Desktop keyboard
     - ``reward-collection`` (``pynput``)
     - .. literalinclude:: ../../examples/install.sh
          :language: bash
          :start-after: # start check-keyboard
          :end-before: # end check-keyboard
   * - SSH terminal / JSONL events
     - No extra needed; the event sources are named ``stdin`` / ``jsonl``
     - .. literalinclude:: ../../examples/install.sh
          :language: bash
          :start-after: # start check-terminal
          :end-before: # end check-terminal
   * - Physical leader arm (read through the device service)
     - Install ``franky``, ``pyagxarm`` or ``piper-sdk`` on the leader host; add ``grpc`` on both ends
     - .. literalinclude:: ../../examples/install.sh
          :language: bash
          :start-after: # start check-grpc
          :end-before: # end check-grpc
   * - Remote device connection
     - ``grpc``, installed on both the device server and client
     - .. literalinclude:: ../../examples/install.sh
          :language: bash
          :start-after: # start check-grpc
          :end-before: # end check-grpc
   * - JPEG image transport
     - ``jpeg``, installed on both ends; not needed for raw transport
     - .. literalinclude:: ../../examples/install.sh
          :language: bash
          :start-after: # start check-jpeg
          :end-before: # end check-jpeg
   * - openpi policy client
     - ``openpi-client``
     - .. literalinclude:: ../../examples/install.sh
          :language: bash
          :start-after: # start check-openpi
          :end-before: # end check-openpi

**What you should see**: import commands print ``OK`` or a version number. HID and event-device commands print device lists;
``tty`` prints the current terminal path. Lists may be empty when no physical device is attached.
These commands only check dependencies or enumerate devices. Check actual readings in the device chapters.

**If it doesn't**:

* ``ModuleNotFoundError``: rerun the installation command in the active environment with the corresponding extra.
* The SpaceMouse list is empty or opening fails: check USB and HID permissions. Continue with :doc:`spacemouse`.
* GELLO serial-port, pedal-permission or leader–follower configuration problems: see :doc:`gello` for further checks.
* ``pynput`` reports a display-server error over SSH: use ``stdin`` instead. Pico data-source check material is temporarily kept in :doc:`gello`.

4. Run a check without hardware
--------------------------------------------------------------------------------

Run the mock device check to confirm that the current environment can import the project, find its commands and read device feedback.

A ``station`` is a YAML file describing which devices this machine has and how to connect to them.
``mock`` means replacing real devices with simulated ones.
``doctor`` generates a check report, with one check per row.
The installation script already ran a mock check if no station was supplied. Run it separately now so you can repeat it later.

.. tutorial-test: skip
.. literalinclude:: ../../examples/install.sh
   :language: bash
   :lines: 24-24

.. literalinclude:: ../../examples/install.sh
   :language: bash
   :lines: 25-26

On machines with ``learning`` installed, also run:

.. tutorial-test: skip
.. literalinclude:: ../../examples/install.sh
   :language: bash
   :start-after: # start learning-import
   :end-before: # end learning-import

**What you should see**: the import command prints ``Imports OK:`` and local paths.
``rwrl --help`` starts as follows. This is an excerpt from actual output:

.. literalinclude:: ../../examples/rwrl-help.txt
   :language: text
   :lines: 1-8

The complete mock report is shown below; machine-dependent values are placeholders:

.. doctor-example: mock
.. literalinclude:: ../../examples/mock-doctor.txt
   :language: text

Each row has the format ``STATUS id: message``. Non-PASS rows append ``hint:`` when a hint is available.
Terminals that support color use the colors below. Pipe, file and JSON output has no color.
Setting ``NO_COLOR=1`` also disables color (the presence of the ``NO_COLOR`` environment variable is enough).

.. list-table:: Check statuses
   :header-rows: 1

   * - Status
     - Meaning
     - Color
     - Blocks a pass
   * - ``PASS``
     - Checked and healthy
     - Green
     - No
   * - ``WARN``
     - Read the hint
     - Yellow
     - No
   * - ``SKIPPED``
     - Not checked; does not prove availability
     - Gray
     - No
   * - ``FAIL``
     - Checked and faulty
     - Red
     - Yes
   * - ``BLOCKED``
     - Cannot check or repair, for example because a device is busy or permissions are insufficient
     - Magenta
     - Yes
   * - ``PENDING_ACTION``
     - The operator still needs to log in again, replug a device or take another action
     - Yellow
     - Yes

Mock skips real network checks. ``host.python`` should point to the interpreter in your chosen project environment;
the default installation in this chapter uses ``.venv/bin/python``, but a custom venv or non-base conda path is also valid.
If that row is WARN, check the environment first. ``Report:`` gives this run's report directory; keep its ``report.json`` for troubleshooting.
The host environment and report directory vary by machine.

**If it doesn't**:

* ``rwrl`` or the project package is not found: confirm that you activated the ``.venv`` created during installation.
* ``rl_garden`` cannot be imported: rerun the installation command with ``learning`` and check that the submodule is complete.
* ``handshake`` reports an unwritable lock directory: check permissions on the user's cache directory. Use ``XDG_CACHE_HOME`` to specify a writable directory.
* Other ``FAIL`` rows remain: follow each row's ``hint`` and rerun. Keep the printed report directory.

You can now continue to :doc:`station` to connect real devices.

.. note::

   **What needs confirmation on hardware in this chapter**

   This chapter checked software imports, command entry points and mock feedback. FCI networking, SDK and controller-box versions,
   HID permissions and readings, the GELLO bus and event devices, the Pico stream, gripper feedback, camera images,
   teleop calibration, control timing and motion boundaries need confirmation with the devices on site.
