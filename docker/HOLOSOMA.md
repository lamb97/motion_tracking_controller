# Holosoma WBT policies in the existing ROS2/MuJoCo controller

Validated with the bundled `ppo_g1_29dof_dancing.onnx` from the local Holosoma
checkout (`bccd4d7`) on 2026-09-20. No policy training or weight changes are needed.

## Convert once

Use a Python environment with `numpy` and `onnx` (the existing `beyondmimic`
environment has both):

```bash
cd /home/lasserre/Yang/motion_tracking_controller
/home/lasserre/miniforge3/envs/beyondmimic/bin/python scripts/convert_holosoma_onnx.py \
  /home/lasserre/Yang/holosoma/src/holosoma_inference/holosoma_inference/models/wbt/ppo_g1_29dof_dancing.onnx \
  /home/lasserre/Yang/holosoma_deploy/ppo_g1_dancing_adapted.onnx
```

This writes an adapted model and its `.initial_pose.json` sidecar, preserving the
original model. It checks the ONNX graph and compares actions and reference outputs
against the original graph at four input/timestep combinations, including clipping.

## Play

```bash
cd /home/lasserre/Yang/motion_tracking_controller
./docker/play_holosoma.sh /home/lasserre/Yang/holosoma_deploy/ppo_g1_dancing_adapted.onnx
```

Uses the existing Docker image, GPU 1 for rendering and CPU inference. The Docker
launcher mounts the source tree, so the launch-file changes are immediately used.
Ctrl+C in the launching terminal or `docker stop beyondmimic-sim2sim` stops playback.

## Compatibility details and observed behavior

- Preserves Holosoma's alphabetically ordered 154-dimensional actor observation:
  actions, base angular velocity, relative joint positions, joint velocities,
  motion command, reference orientation.
- Maps joint names, full-precision gains, default joint angles and per-joint action
  scales from the ONNX's embedded training configuration into controller metadata.
- Converts the reference quaternion XYZW to WXYZ and exposes the torso reference
  as a one-body motion. It does not fabricate references for other body parts.
- Preserves trained observation/action clipping. Unsupported observation terms,
  histories and per-term scales are rejected rather than guessed.
- Starts the simulation joints at reference frame zero using optional URDF
  state-interface initial positions. This is distinct from the default joint
  angles used for policy observations and action offsets. Root pose and velocities
  still use the simulator's defaults; this is not a full reference-state reset.
- Loads the active state estimator and policy before spawning the inactive
  standby controller. Parallel spawners could otherwise delay policy takeover
  by approximately three seconds while the simulator was already running.
- The default simulator joint initialization fell during the first test. With
  frame-zero joint initialization, the policy visibly danced and remained upright
  over approximately 100 simulated seconds. The user also confirmed working
  playback. A restart exposed the spawner ordering issue described above.
  After making startup sequential, a background Xvfb run covered simulation time
  0.002–145.012 seconds, including the full clip. Pelvis IMU orientation sampled at
  approximately 1 Hz had a maximum tilt of 28.09 degrees, with no observed fall.
  This checks gross stability, not quantitative motion-tracking accuracy. The
  IMU topic's position fields are zero and were not used for validation. Logs and
  the summary are in `/home/lasserre/Yang/holosoma_deploy/dance_validation_sequential*`.
  One earlier viewer
  session exited with a simulator segfault while adjusting the view; no root cause
  has been established.

The embedded clip contains 6574 frames (about 131.5 seconds at 50 Hz). At its end,
the ONNX reference clamps to the last frame; it does not automatically restart.
Restart the launch to replay from frame zero. Do not use the viewer's reset as a
substitute for resetting both the physics state and the controller's motion clock.

This validates the PPO dance example. FastSAC and object-interaction policies have
not been tested here. Object tasks additionally require matching object physics,
robot collision geometry and synchronized initialization in the MuJoCo scene.
