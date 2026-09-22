# Docker sim2sim environment

Runs ROS 2 Jazzy on Ubuntu 24.04 with the author's MuJoCo and legged-control
packages, without installing ROS into the host or the training Conda environment.
Requires Docker with the NVIDIA Container Toolkit and an X11 desktop session.

## Build

From the repository root:

```bash
./docker/build.sh
```

Image: `beyondmimic-sim2sim:jazzy`. `unitree_bringup` is pinned to a commit in
the Dockerfile. Binary dependencies come from the upstream Noble/Jazzy buildfarms
described in <https://qiayuanl.github.io/legged_control2_doc/installation.html>.
Their versions can change on future clean builds.

The author's `mujoco_ros2_control` package is explicitly preferred over the
unrelated package with the same name in the official ROS repository.

## Open the simulation

Run in a terminal in the graphical desktop. Conda activation is not required.

```bash
cd /home/lasserre/Yang/motion_tracking_controller
./docker/play.sh \
  /home/lasserre/Yang/whole_body_tracking/logs/rsl_rl/g1_flat/2026-09-17_18-29-13_walk_full_v1/exported/policy.onnx
```

The launcher uses GPU 1 for rendering and CPU ONNX inference. It automatically
starts the state estimator and motion controller. Joystick teleoperation is
disabled by default because no gamepad is attached. Extra ROS launch arguments
can be appended, for example `start_step:=100`.

Stop with Ctrl+C in the launching terminal, or from another terminal:

```bash
docker stop beyondmimic-sim2sim
```

The container is removed on exit; the image and host policy remain available.
Only one container with this default name can run at a time.

`BEYONDMIMIC_GPU`, `BEYONDMIMIC_IMAGE`, and `BEYONDMIMIC_CONTAINER` override
the GPU index, image, and container name. Use a GPU capable of rendering to the
current desktop. On the validated machine this is GPU 1.

The launcher mounts the host Xauthority file and the matching NVIDIA GPU compiler
library read-only. The latter addresses a missing driver library in this host's
container runtime; without it OpenGL fell back to software rendering. It does not
change the host driver or disable X authentication.

The source tree is mounted read-only to support the build's symlink installation.
Launch/config edits apply on the next run; C++ edits require rebuilding the image.
ROS discovery is confined to localhost inside the container, using domain 73.

## Validation on 2026-09-20

- Both `unitree_bringup` and `motion_tracking_controller` compiled successfully.
- MuJoCo 3.2.7 opened the G1 model with NVIDIA hardware OpenGL rendering.
- The exported walking ONNX loaded, with 160 observations and 29 actions.
- `state_estimator` and `walking_controller` reported `active`.
- The first runtime test fell. Subsequent walking playback was visually confirmed
  by the user after setup corrections; no quantitative walking tracking score was
  recorded. For the Holosoma dance adapter and its full-clip stability check, see
  [HOLOSOMA.md](HOLOSOMA.md). These checks do not establish real-robot readiness.

To inspect controller status while running:

```bash
docker exec beyondmimic-sim2sim \
  /beyondmimic-entrypoint.sh ros2 control list_controllers
```
