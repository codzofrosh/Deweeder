# Mock ROS launch acceptance — 2026-10-20

**Runtime acceptance passed. Package lint did not pass.** Two runtime defects exposed by installed-package testing were fixed; no physical driver was enabled.

| Verification | Final result |
|---|---|
| Colcon build: `weedbot_msgs`, `weedbot_core` | Both built and installed |
| Installed mock acceptance runner | 3 tests passed, 19 recorded checkpoints, 0 skips; 38.367 s |
| Full repository regression suite with ROS | 205 passed, 0 skips; 12.435 s |
| Colcon package tests using pytest | 2 failed (flake8, pep257), 1 existing copyright skip |
| Launch shutdown | Four nodes exited cleanly; no traceback or killed child; parent exit0 |

Environment: Ubuntu24.04 WSL2, ROS Jazzy, Fast DDS (`rmw_fastrtps_cpp`). Acceptance ran in localhost-only domain47; regression used domain48. Colcon tooling was installed in WSL. Builds are under `~/deweeder_launch_validation`, outside the source tree. The acceptance runner imports the installed packages without adding the core source directory to Python's import path.

## Executed checks

The real `robot_core.launch.py` ran robot-state, safety, mock HAL and motor-controller processes, with `use_behavior:=False`, `hw_mode:=False`, `hw_backend:=mock`. The harness waited for DDS discovery and received diagnostics before checking startup.

| Scope | Checks and evidence |
|---|---|
| Complete launch | Missing sensors hold STOP and reject arming; valid synthetic packets permit explicit arming; emergency stop latches; release does not automatically restart; extreme tilt rejects safety reset |
| Complete launch input failures | Independently stop the raw sensor packet, heartbeat, motion-command and tool-command streams while mock motion/tools are commanded active; `/hal/ready` becomes false and diagnostics identify faults |
| Publisher process termination | Terminate a separate heartbeat publisher process while the remaining streams continue; observe HAL disarm |
| Installed HAL with real DDS in an executor | Establish nonzero mock wheel output and all three tool channels active before independently dropping heartbeat, safety, feedback, motion and tool streams; verify all four PWM outputs and all tool outputs become zero/off |
| Installed HAL emergency stop | Active wheel/tool outputs stop; emergency release alone does not restart them |
| Frozen ROS time | Set `use_sim_time=True` with no `/clock` advancement (ROS time stays zero). HAL heartbeat watchdog and separate safety-node contact/IMU freshness timers still stop; stale inputs reject reset |
| Shutdown | Destroy active installed HAL and verify final mock wheel/tool outputs off and driver unhealthy; separately signal the actual launch parent and verify clean exits of all four nodes |

The subprocess launch checks observe public readiness/diagnostic topics; they do not expose its private mock-driver arrays. Final driver outputs are asserted in the separate installed-HAL/DDS test. Synthetic encoder feedback is an injected fixture, not a simulated physical plant. The frozen-clock checks do not run Gazebo or demonstrate a paused/resumed simulator. The observation windows are test deadlines, **not measured or guaranteed physical stop-time bounds**.

## Defects fixed

1. `SafetyNode._imu_cb`: generated ROS covariance arrays yielded a NumPy `bool_` from comparisons. `SafetyMonitor` correctly requires an actual boolean, so healthy ROS IMU data was incorrectly rejected. Convert the completed validity check with `bool(valid)` before passing it to the monitor. Added a regression test covering valid covariance, unavailable covariance and NaN covariance. Actual whole-launch sensor delivery also passed after the correction.
2. The four launched nodes called `rclpy.shutdown()` after the signal handler had already shut down the context, producing exit1 tracebacks. Their cleanup now uses `rclpy.try_shutdown()` after node destruction. The HAL's driver-stop-before-destruction behavior remains intact. The actual SIGINT launch test now exits cleanly.

The initial runner also needed a DDS-discovery wait and a single signal to the launch parent, rather than signaling both the parent and its children. Those harness issues were corrected before evaluating the final shutdown result. Failed attempt logs are preserved and are not counted as passing runs.

## Remaining package failures

Colcon's default Python test selection initially ran zero tests and returned a failure for `weedbot_core`; zero tests were not treated as acceptance. Explicit `--python-testing pytest` executed the existing test files and produced:

- flake8:107 findings (layout, line length, unused imports and whitespace).
- pep257:31 docstring findings.
- Copyright: existing skip because generated source headers have not been completed.

These pre-existing package hygiene checks remain unresolved; no blanket ignores, disabled tests or invented copyright attribution were introduced. Thus the **overall package test gate is still failing**, despite passing runtime acceptance. The next software cleanup is to resolve these findings and rerun the package gate. It does not authorize new ROS features, CAD work or powered testing.

## Reproduction

Enter Ubuntu using `wsl -d Ubuntu-24.04`, then:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/deweeder_launch_validation
cd ~/deweeder_launch_validation
colcon build --base-paths /mnt/c/Users/rosha/Deweeder/deweeder_tertill/src \
  --packages-select weedbot_msgs weedbot_core --executor sequential
source install/setup.bash
colcon test --base-paths /mnt/c/Users/rosha/Deweeder/deweeder_tertill/src \
  --packages-select weedbot_msgs weedbot_core --python-testing pytest \
  --return-code-on-test-failure
colcon test-result --verbose
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=47
export ROS_ACCEPTANCE_OUT="$HOME/deweeder_launch_validation/acceptance_replay"
python3 /mnt/c/Users/rosha/Deweeder/tools/validate_ros_launch.py
cd /mnt/c/Users/rosha/Deweeder
python3 -m unittest discover -s tests -v
```

Use an otherwise unused domain. The acceptance runner launches and tears down its own processes. It asserts hardware remains on the mock launch path; do not adapt this command to enable physical drivers.

Evidence: [structured results and source hashes](validation/ros2_launch_2026-10-20/results.json), [runtime observations](validation/ros2_launch_2026-10-20/observations.json), [launch log](validation/ros2_launch_2026-10-20/launch.txt), [regression log](validation/ros2_launch_2026-10-20/final_full_suite.txt), [package failures](validation/ros2_launch_2026-10-20/final_colcon_results.txt), [acceptance runner](../tools/validate_ros_launch.py).

Simulator integration, independent hardware watchdog, hardwired emergency interruption, actuator channel mapping and physical stop/coast times remain unverified. The problem-validation roadmap and freeze on new features remain unchanged.

Repository scope: the 205-test count describes the full local workspace captured by the source hashes. Additional study/configuration modules in that workspace are outside this ROS-fix commit, so a clean remote checkout may discover fewer root-suite tests. The installed ROS acceptance runner and its ROS package dependencies are included. Raw build logs retain original whitespace for evidence integrity.
