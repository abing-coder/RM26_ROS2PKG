# Repository Guidelines

## Project Structure & Module Organization
RM26_ROS2PKG is a ROS 2 Humble workspace: core packages live in `src/` and build outputs land in `build/` and `install/`. `src/recive_pkg` owns the ROS nodes that subscribe to `/image_raw` and publish `target_delta`, while `src/rm_auto_aim/detection` houses the OpenVINO pipeline and its ONNX models in `model/0526.onnx`. Hardware drivers live in `src/ros2_hik_camera-main` (camera configs in `config/camera_params.yaml`) and `src/ros2_armor_can`. Helper scripts such as `launch_motor.sh`, `kill_nodes.sh`, and `incremental_build.sh` sit at the workspace root; performance logs accumulate in `log/`.

## Build, Test, and Development Commands
- `source /opt/intel/openvino_2024.6.0/setupvars.sh && source install/setup.bash` to load toolchains before running nodes or tests.
- `colcon build --symlink-install` compiles every package, while `colcon build --packages-select recive_pkg` or `--packages-select detection` keep iterative builds tight.
- `./incremental_build.sh` detects changed YAML configs and rebuilds affected packages automatically.
- `ros2 launch ros2_armor_can motor_all.launch.py` starts the full perception-to-CAN stack; use `ros2 run recive_pkg recive_pkg` or `bash launch_motor.sh` for ad-hoc bringup.

## Coding Style & Naming Conventions
All C++ nodes target C++17 via `ament_cmake`; keep headers in `include/<pkg>/` and mirror source paths in `src/`. Use four-space indentation, brace-on-same-line, and `snake_case` for functions, ROS topic names, and frames (e.g., `target_delta`), while classes and structs remain `PascalCase` (`DetectionArmor`). Prefer `rclcpp::Node` composition, avoid raw pointers, and sync parameter names with YAML (e.g., `exposure_time`). When editing formatting-heavy blocks, run `clang-format` or `ament_uncrustify` locally before committing.

## Testing Guidelines
Unit and integration checks run with `colcon test --packages-select <pkg>`; ensure each new node adds at least a smoke test or launches under `ros2 test`. Use `bash test_aim.sh` to execute the offline detector replay via `build/detection/test_aim_node`, and attach representative video snippets in `log/`. For hardware HIL tests, capture `ros2 topic echo target_delta` output and attach CAN bus traces. Contributions should keep detection accuracy regressions under 2% and document any new parameters inside the relevant YAML.

## Commit & Pull Request Guidelines
The history mixes Chinese descriptions with conventional commits such as `feat(detection): 将预处理迁移到 OpenVINO...`; follow the `type(scope): imperative summary` template and keep lines under 72 chars (e.g., `fix(camera): clamp exposure bounds`). Reference related issues (`#123`) and mention affected launch files or configs. Pull requests target `TX`, include a concise summary, testing evidence (command snippets, screenshots of detection overlays, CAN logs), and note whether scripts like `kill_nodes.sh` need updates. Tag reviewers familiar with the touched package and wait for CI `colcon` + `test_aim.sh` logs before merging.
