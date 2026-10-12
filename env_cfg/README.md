# Legacy RoboDojo X5 metadata

The published XPolicyLab VPP2 adapter at revision `125d5de` still calls
`get_robot_action_dim_info("arx_x5")`, which reads this parent directory.
Retain the existing `arx_x5.yml`, `robot/_robot_info.json` and
`sim/sim_config.yml` for that active deployment. These files describe dimensions
and legacy batch metadata; they do not connect to simulation or hardware.

New deployment recipes own explicit `robot_action_dim_info` and `num_envs`
and pass the resolved config to XPolicyLab's shared helpers. This directory is
a bounded compatibility dependency, not a registry for new policies.
VPP2 must migrate its lookup in its owning framework before these files can
be removed. Physical station bindings belong in `manimux/configs/local/`.
