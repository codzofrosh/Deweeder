# ROS 2 WSL validation — 2026-10-19

Mock launch verification update (2026-10-20): both ROS packages built; installed mock acceptance passed 3 tests/19 checkpoints; regression passed 205 tests with zero skips. Fixed generated-IMU boolean validation and duplicate ROS shutdown calls. Package lint remains failing (2 failures, 1 existing skip). See [full results and limits](ros2_mock_launch_acceptance.md). No powered-test or feature-freeze gate is lifted.


**The previously skipped HAL ROS/DDS integration test executed and passed. The complete repository suite then passed all 204 tests, with zero skips.**

Environment: Ubuntu-24.04 on WSL 2, ROS 2 Jazzy, system Python, `rmw_fastrtps_cpp`. Tests used `ROS_DOMAIN_ID=43` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. No interrupted test processes were found before execution. The HAL constructs `MockDriver`; no physical driver or powered mechanism was started.

The integration test `test_dds_emergency_stop_service_rearm_and_timer_expiry` exercised real DDS subscriptions, the `/hal/arm` service and the steady-clock watchdog. It verified startup disarmed, motion after explicit arming, emergency-stop latching and zero wheel outputs, no automatic restart after emergency release, explicit re-arm, and zero wheel/tool outputs after command publication ceased. Its tool commands are neutral; this test alone does not demonstrate active-tool stopping across every source-loss condition.

| Check | Result |
|---|---|
| Generate/build/install existing `weedbot_msgs` | PASS |
| Deferred ROS integration test alone | 1 passed, 0 skipped; 3.426 s |
| Complete existing unittest suite with ROS enabled | 204 passed, 0 skipped; 10.883 s |

`colcon` was unavailable, so only the required message package was built with CMake into `~/deweeder_ros2_validation`. The test imports core Python source directly. This does **not** establish installation or launch correctness of the complete `weedbot_core` package. No application code changes were needed.

## Reproduce in Ubuntu WSL

From PowerShell, enter `wsl -d Ubuntu-24.04`. Then:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S /mnt/c/Users/rosha/Deweeder/deweeder_tertill/src/weedbot_msgs \
  -B ~/deweeder_ros2_validation/build/weedbot_msgs \
  -DCMAKE_INSTALL_PREFIX="$HOME/deweeder_ros2_validation/install" \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build ~/deweeder_ros2_validation/build/weedbot_msgs -j 2
cmake --install ~/deweeder_ros2_validation/build/weedbot_msgs
source ~/deweeder_ros2_validation/install/share/weedbot_msgs/local_setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=43
cd /mnt/c/Users/rosha/Deweeder
python3 -m unittest discover -s tests -p test_hal_ros_integration.py -v
python3 -m unittest discover -s tests -v
```

Evidence: [structured results and source/log hashes](validation/ros2_wsl_2026-10-19/results.json), [integration log](validation/ros2_wsl_2026-10-19/hal_ros_integration.txt), [full-suite log](validation/ros2_wsl_2026-10-19/full_suite.txt). Configure/build/install logs are retained in the same folder. The report date follows the user's client date; raw runtime timestamps are retained unchanged.

Still unexecuted: colcon package lint/test targets; whole `robot_core.launch.py` acceptance; active-tool independent source-loss and paused-simulation-time launch scenarios; Gazebo/Webots integration; physical stopping/watchdog verification. Historical Windows-only reports remain valid records of their earlier skips. This result supersedes only the claim that the existing HAL integration test has never executed. It does not release powered testing, change the problem-validation roadmap, or resume frozen feature/CAD work.
