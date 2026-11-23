# Third-party notices and acknowledgements

real-world-rl builds on the work of several open-source projects. This file lists
the code that is ported, derived from or vendored into this repository, the
licence each part carries, and the papers to cite. Ported files keep their
original copyright and licence notices in their headers.

## Ported and derived code

| Project | Licence | Revision | What this repository uses | Where |
| --- | --- | --- | --- | --- |
| [serl_franka_controllers](https://github.com/rail-berkeley/serl_franka_controllers) (RAIL Berkeley) | MIT, Copyright (c) 2024 RAIL Berkeley | `1f140ef` | The Cartesian impedance controller, its `compliance_param.cfg` parameter table, `pseudo_inversion.h` and the joint position (reset) controller, ported to C++ without ROS and run inside franky's 1 kHz loop | `native/serl_controller/` (`include/`, `src/`; licence in `native/serl_controller/LICENSE`) |
| [SERL](https://github.com/rail-berkeley/serl) (Luo et al.) | Apache-2.0, Copyright 2024 Jianlan Luo, Zheyuan Hu, Charles Xu, You-Liang Tan | `1fa2af7` | Robot-server behaviour (`franka_server` controller lifetime, gripper server, SpaceMouse expert), RelativeFrame conventions, the legacy controllers and teleop | `real_world_rl/legacy/`, `real_world_rl/devices/backends/arms/franky.py` (serl mode), `real_world_rl/envs/rlinf_cartesian.py` (RelativeFrame adjoint) |
| [HIL-SERL](https://github.com/rail-berkeley/hil-serl) (Luo et al.) | Apache-2.0, Copyright 2024 Jianlan Luo, Charles Xu, Jeffrey Wu | `c32939b` | SpaceMouse intervention (raw base-frame twist, button levels), the vendored PySpaceMouse 1.0.3 device table, the Franka Hand values of `franka_gripper_server`, the joint reset polling of `franka_server.reset_joint`, the controller parameter profiles, `grasp_penalty` | `real_world_rl/teleop/spacemouse/{serl,hid}.py`, `real_world_rl/devices/backends/grippers/franky.py`, `real_world_rl/devices/backends/arms/franky.py`, `real_world_rl/envs/wrappers/grasp_penalty.py` |
| [PySpaceMouse](https://github.com/JakubAndrysek/PySpaceMouse) (John Harrison et al.) | MIT | 1.0.3 (as vendored by HIL-SERL) | SpaceMouse HID device table and report decoder | `real_world_rl/teleop/spacemouse/hid.py` |
| [RLinf](https://github.com/RLinf/RLinf) (The RLinf Authors) | Apache-2.0 | checked against `cc6bd25` | The single-arm `RelativeFrame` / `FrankaEnv` Cartesian action and state conventions (`cartesian_semantics: rlinf`), the `FrankyGripper` open threshold, the Gymnasium registration contract of RLinf's real-world env worker and the RLinf Franka openpi deploy profile | `real_world_rl/envs/rlinf_cartesian.py`, `real_world_rl/integrations/rlinf/`, `configs/rlinf/`, `real_world_rl/deploy/openpi/profiles.py` |
| [GELLO](https://github.com/wuphilipp/gello_software) `franka_fr3_arm_controllers` (Franka Robotics GmbH) | Apache-2.0, Copyright (c) 2025 Franka Robotics GmbH | `9fc8c26` | The joint impedance law (K·(q_d − q) − D·dq_filtered), the joint-velocity low-pass filter and the default gains, filter constant and command timeout; ported into `native/joint_controller` (the quintic reference planner, timeout stop, coriolis term and torque-rate limit are this repository's own) | `native/joint_controller/` (`src/law.cpp`, `include/joint_controller/params.hpp`; licence in `native/joint_controller/LICENSE` and `LICENSE.Apache-2.0`) |

## Libraries used at run time

| Project | Licence | Notes |
| --- | --- | --- |
| [franky](https://github.com/TimSchneider42/franky) (Tim Schneider, Nick Walker, Lars Berscheid) | MIT (from 2.0.0; earlier releases LGPL-3.0) | The Franka backend. `native/serl_controller` and `native/joint_controller` link against an **unmodified** franky v2.0.0 (`e7e63c7`) and derive from its `Motion` classes; the wheels are built together from franky's own, unmodified build recipe. |
| [libfranka](https://github.com/frankaemika/libfranka) (Franka Robotics GmbH) | Apache-2.0 | Bundled inside the franky wheel. |
| [Eigen](https://eigen.tuxfamily.org) | MPL-2.0 | Header-only, used by `native/serl_controller` and `native/joint_controller`. |
| [pybind11](https://github.com/pybind/pybind11) | BSD-3-Clause | `native/serl_controller` and `native/joint_controller` Python bindings. |

## Vendored for tests only

These files are copied verbatim (byte-identical, checked by `SHA256SUMS`) so the
golden tests can compile the originals next to the port. They are not part of
any built package and are excluded from lint and formatting.

| Project | Licence | Revision | Files |
| --- | --- | --- | --- |
| serl_franka_controllers | MIT | `1f140ef` | `native/serl_controller/tests/cpp/golden/serl_original/` |
| [ros_control](https://github.com/ros-controls/ros_control) `joint_limits_interface` | BSD-3-Clause | noetic-devel `633c374` | `native/serl_controller/tests/cpp/golden/ros_control_original/` |
| [franka_ros](https://github.com/frankaemika/franka_ros) (`franka_hw`, `franka_control`, `franka_description`) | Apache-2.0 | `ddd2fff` | `native/serl_controller/tests/cpp/golden/franka_ros_original/` |

Each directory carries its project's own `LICENSE`.

## Citing

If this repository's SERL controller or SpaceMouse intervention helps your work,
please cite SERL and HIL-SERL; if you use the RLinf integration, please cite RLinf.
The SERL and HIL-SERL entries are copied verbatim from the Citation sections of their repository READMEs
(`1fa2af7`, `c32939b`).

```bibtex
@misc{luo2024serl,
      title={SERL: A Software Suite for Sample-Efficient Robotic Reinforcement Learning},
      author={Jianlan Luo and Zheyuan Hu and Charles Xu and You Liang Tan and Jacob Berg and Archit Sharma and Stefan Schaal and Chelsea Finn and Abhishek Gupta and Sergey Levine},
      year={2024},
      eprint={2401.16013},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}

@misc{luo2024hilserl,
      title={Precise and Dexterous Robotic Manipulation via Human-in-the-Loop Reinforcement Learning},
      author={Jianlan Luo and Charles Xu and Jeffrey Wu and Sergey Levine},
      year={2024},
      eprint={2410.21845},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}

@inproceedings{yu2026rlinf,
  author    = {Chao Yu and Yuanqing Wang and Zhen Guo and Hao Lin and Si Xu and Hongzhi Zang and Quanlu Zhang and
               Yongji Wu and Chunyang Zhu and Junhao Hu and Zixiao Huang and Mingjie Wei and Yuqing Xie and Ke Yang and
               Bo Dai and Zhexuan Xu and Jiakun Du and Xiangyuan Wang and Xu Fu and Letong Shi and Zhihao Liu and
               Kang Chen and Weilin Liu and Gang Liu and Boxun Li and Jianlei Yang and Zhi Yang and Guohao Dai and Yu Wang},
  title     = {{RLinf}: Flexible and Efficient {Large-Scale} Reinforcement Learning via {Macro-to-Micro} Flow Transformation},
  booktitle = {20th USENIX Symposium on Operating Systems Design and Implementation (OSDI 26)},
  year      = {2026},
  address   = {Seattle, WA},
  pages     = {829--846},
  publisher = {USENIX Association},
  url       = {https://www.usenix.org/conference/osdi26/presentation/yu-chao}
}
```

The RLinf entry is the one RLinf's README asks for; RLinf-USER
(arXiv:2602.07837) is the RLinf paper for real-world online policy learning.
franky asks to be cited through its `CITATION.cff`.
