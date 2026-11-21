Verbatim copies (golden test / provenance only) from frankaemika/franka_ros @ ddd2fff,
Apache-2.0 (LICENSE): the panda / fr3 joint limits and the `franka-limits` xacro macro that
turns them into URDF `<limit>` + `<safety_controller k_position="100" soft_*_limit=...>`
(franka_description), franka_hw's `setupLimitInterface` / `enforceLimits` / `reset`
(franka_hw_src/), and franka_control_node's loop that calls them (reset on the period-0
tick, enforceLimits after every other controller update). Checksums: SHA256SUMS.
