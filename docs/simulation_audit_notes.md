# Phase 0 simulation and legacy ROS audit notes

Audit date: 2026-09-14. Scope: root `src/`, Webots assets, vision artifacts, root launcher. Read-only source inspection and parsing only; no simulation, training, actuation, dependency installation, or source edits performed. These notes support the consolidated audit and architecture proposal.

## Architecture and disposition

| Component | Existing role | Disposition |
|---|---|---|
| `controllers/my_controller/my_controller.py` | Standalone Webots HSV red-object detection, zigzag mobility, two-joint arm animation | FREEZE navigation/vision/arm behavior; KEEP as educational reference, not a validated controller |
| `worlds/Farm of Deweeder.wbt` | Webots R2025a scene, 2 m square visual floor, five crops, three red weeds, E-puck and separate arm robot | KEEP visual primitives; REFACTOR into optional geometry visualization only when needed |
| `protos/RectangleArena.proto`, icon | Cyberbotics arena template; not referenced by the checked-in world | KEEP unused reference; retain its Webots-only asset license header |
| `src/deweeder_robot` | Legacy ROS Python HSV detector commanding `/cmd_vel` and `/weed_detected`; arm subscriber emits joint trajectories | FREEZE arm/vision stream; record build/launch defects before any reactivation |
| `src/weedbot_simulation_tertill` | Gazebo Tertill body, four wheels, front/belly contact sensor model, field world and launch | KEEP reusable body/contact descriptions; REFACTOR parse, axes, packaging and launch defects if Gazebo regression work is needed |
| `start_simulation.sh` | Six-terminal launcher for the legacy tracked robot | FREEZE legacy workflow; replace later only if needed for a supported launch path |
| `deweeer_yolo_training.py`, `Deweeder_Yolov8.ipynb` | Colab YOLO experiment, checkpoint, training and export code | FREEZE; do not delete or train |
| `crop_field.world`, `crop_field_utf8.world` | Two closely related fixed crop-row visualization scenes | UNKNOWN canonical version; differing hashes, not proven byte duplicates; no deletion |

No root ROS package connects Webots to the Tertill core. Root `deweeder_robot` actually launches Gazebo despite the README calling it Webots-integrated. The standalone Webots controller imports neither ROS nor YOLO. No native CAD/STEP/STL/SCAD files or model weights (`.pt`, `.onnx`, `.tflite`) were found by recursive extension scan. Geometry in URDF/WBT is a reusable schematic asset, not a manufacturing drawing.

## Reproducible checks

- PowerShell XML parse of root `src` manifests/models/worlds: **5 PASS, 2 FAIL**. Both package manifests, tracked URDF and two crop worlds parse. Tertill URDF fails at line 292 (conflict marker); Tertill world fails at line 107 (case-mismatched closing element). XML well-formedness does not validate simulator semantics.
- Notebook JSON parse: **PASS**, 11 cells (10 code, one markdown); stored execution outputs exist. They are historical, unreproduced artifacts, not current test results or weed-suppression evidence.
- `requirements.txt` referenced by README line 26 is absent.
- `Get-Command` finds no `webots`, `colcon`, `ros2` or `gz` on PATH in this Windows session. Existing build/launch procedures cannot be executed here as documented without another environment. `bash.exe` exists but this alone does not establish a working Linux ROS environment.
- `rg` cannot execute in this session (Windows reports no file association); inspection used PowerShell recursive file listing and `Select-String` instead.

## Broken or misleading behavior with evidence

| Priority | Evidence | Finding and consequence |
|---|---|---|
| P0 parse | `src/weedbot_simulation_tertill/urdf/weedbot.urdf:292`, `:305`, `:306` | Unresolved conflict markers make URDF invalid XML. |
| P0 parse | `src/weedbot_simulation_tertill/worlds/weedbot.world:107` | `</Visual>` does not match `<visual>` at line 105; world is invalid XML. |
| P1 run | `worlds/Farm of Deweeder.wbt:127`, `:134`, `:224`; controller `:11`, `:15`, `:21` | Same controller runs on two separate Robot nodes. E-puck owns camera/wheels; separate arm owns shoulder/elbow. Neither owns all required devices; controller initialization or arm calls cannot succeed as written. Arm is not attached to moving base. |
| P1 behavior | controller `:80`, `:168`, `:173`, `:185`, `:190` | State timer increments only inside SEARCHING. POSITIONING_ARM resets timer to zero and waits for it to exceed 40 without incrementing it. RESETTING has analogous deadlock. |
| P1 safety | controller `:137`, `:158`; detector `src/deweeder_robot/deweeder_robot/scripts/weed_detector.py:103` | Missing camera frames skip processing without stopping existing wheel commands; lost target during APPROACHING can retain last velocity. ROS image callback exception only logs, also without publishing stop. No independent fail-safe established for this legacy path. |
| P1 integrity | controller `:109`, `:181` | Treatment completion is a timer/animation and counter. No weed removal, plant damage, tool-contact test or biological outcome is measured. Log wording “eliminated” is unsupported. |
| P1 package | `src/deweeder_robot/setup.py:10`, `:33` | Installs only top-level package and entry points `deweeder_robot.weed_detector` / `.arm_controller`; actual modules are under `deweeder_robot/scripts`. Console entry points cannot import targets. |
| P1 launch | `start_simulation.sh:6`, `:7`, `:10`, `:34`; `src/deweeder_robot/launch/deweeder_gazebo.launch.py:50`; world `:3` | Hardcoded Jazzy and `~/deweeder_ws`, gnome-terminal requirements; spawn service `/world/default/create` conflicts with loaded world name `crop_field`. |
| P1 integration | tracked URDF `:158`, `:174`; Gazebo launch `:23`; `config/controllers.yaml` | Classic `libgazebo_ros2_control.so`/`libgazebo_ros_camera.so` assets paired with `gz sim`; controller YAML is empty. No ros2_control hardware/joint interface definitions, no mobile drive system, and tracks are fixed joints (`:50`, `:66`). Functional motion and arm control are unestablished. |
| P1 sensing | tracked URDF `:176`; detector `:22` | Camera plugin declares `/deweeder` namespace but detector expects `/camera/image_raw`; topic wiring requires verification/correction even after simulator compatibility repairs. |
| P1 responsiveness | `src/deweeder_robot/deweeder_robot/scripts/arm_controller.py:50`, `:55`, `:59`, `:70` | Multi-second sleeps in subscriber callback, no completion feedback, no safety subscriber and no target-position data. Repeated true detections can trigger stale treatment sequences. |
| P1 dynamics | Tertill URDF `:47`, `:63`, `:64`, `:97`, `:131`, `:165`, `:312` | Wheel cylinders rotated about X but joint axes remain Z. Wheel labels/grouping mix front/back with left/right: “front right” at negative X, positive Y and “rear left” at positive X, negative Y. DiffDrive groups opposite lateral sides together. Geometry/control axes need review. |
| P2 packaging | `src/weedbot_simulation_tertill/CMakeLists.txt:26`; launch `:16`; manifest `:10` | Installs launch/urdf only, omits worlds; launch starts `gz sim -r` without checked-in world; runtime launch dependencies such as `ros_gz_sim`, `launch_ros`, `robot_state_publisher` absent from manifest. |
| P2 collision | Webots world `:26`, `:41`, `:108` | Visual floor/crops/weeds lack `boundingObject`. No meaningful crop collision or terrain-support evaluation from these shapes. |
| P2 reproducibility | `deweeer_yolo_training.py:1`, `:13`, `:29`, `:226` | File starts with notebook `!pip` syntax and imports Colab; cannot run as standard Python despite README `python deweeer_yolo_training.py` instruction. Dataset/checkpoints on external Drive, not repository. |
| P2 claims | `README.md:8`, `:9`, `:224` | “YOLO-based” Webots and “Production-ready”/“Hardware Integration Ready” are not supported by inspected wiring and validation evidence. |

## Minimum next simulator design (proposal, not implemented)

Build a ROS-independent, seeded 2D geometry evaluator for a 2–3 m controlled-speed test row. Use SI units and record all configuration values, seeds, software revision and output provenance. Generate crop/weed IDs, centres and radii; model crop height/tool depth as declared envelope metadata and exclusion rules, never soil-destruction physics.

Required inputs: row_spacing, plant_spacing, plant_spacing_variance (explicit variance units m²), crop_radius, crop_height, weed_density (plants/m²), weed_distribution (uniform/clustered/row-biased), weed_radius, robot_lateral_error, robot_heading_error, tool_width, tool_depth, tool_active_radius, tool_response_time, vehicle_speed. Add row length/count, guard clearance, longitudinal error, response-time variability, geometry-specific stroke/frequency or tine radius, missing-crop probability and random seed. Explicitly define whether each error value denotes a fixed bias, standard deviation or bound; include correlated drift, not only independent noise each step.

Tool adapters return time-varying active footprints and passive guard footprints: circular/string sweep, narrow oscillating blade sweep, shallow tine disturbance envelope. Controlled linear motion or a manually supplied trajectory suffices. Identical generated rows and error realizations compare concepts fairly. Account for on/off delay; along-path timing offset is velocity times delay. Separate true crop geometry from any nominal/sensed crop estimate used by clearance control, so the simulator cannot accidentally grant perfect crop knowledge.

Use analytic swept intersections or step-size convergence checks to prevent missed contacts at high speed. Record unique crop contact and weed intersection separately from treatment-event counts. Outputs are **SIMULATED geometric weed intersection probability** and **SIMULATED crop contact probability**, not WRE/CDR: no biological efficacy has been demonstrated. WRE and damage remain unavailable until bench data defines outcomes. Minimum safe clearance is initially a geometric margin; no agronomic safety claim follows.

CSV/JSON event record: experiment/run/seed/config ID, timestamp, vehicle/tool pose, minimum crop and weed boundary distance, commanded and actual actuator state, contact entity ID, collision flag, treatment-intersection flag. Summary includes denominators, unique-plant rates, missed weeds, actual elapsed traversal time, metres/minute and theoretical capacity. Zero denominators should be null with explanation. Energy/cost must remain null without declared measured/estimated power and cost inputs. Plots show crops/weeds, guard zones, swept trajectory, contacts and misses.

Initial sweep uses speeds 0.1/0.25/0.5/0.75/1.0 m/s, lateral errors 0/5/10/20/30/50 mm and safety margins 20/30/40/50/75 mm. Distinguish crop exclusion margin from cutting radius, then vary tool widths and response delays from mechanically justified bounds. Run an initial deterministic boundary suite and small seeded smoke sweep before scaling to 10,000 rows per combination. Report plant- and run-level denominators; rows are independent sampling units and within-row plants may be correlated, so uncertainty should reflect clustering. No Monte Carlo results have been produced during Phase 0.

QA cases: exact tangency, zero weeds/crops, missing crop, unusually close crops, crop/weed overlap, dense weed clusters, invalid/negative dimensions, unit mistakes, delayed off command, drift and heading error, repeated event deduplication, sensor loss, stale commands, and tool running after vehicle stops. Full motor/encoder/tilt/e-stop behavior belongs in safety/control tests, not invented plant physics. Keep Webots as an optional visualization and integration adapter after pure geometry checks pass.
