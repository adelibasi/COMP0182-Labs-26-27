# Lab 5 — SLAM: Mapping in Simulation and on the Real Robot

In Lab 3, you read LiDAR and odometry data and explored the TF frame tree. In Lab 4, you used odometry feedback to control the TurtleBot3. Now you'll combine those ideas to build a map of an unfamiliar area, then localise against the saved map.

**SLAM** stands for *simultaneous localisation and mapping*: the robot estimates where it is while mapping its surroundings. We'll use **RTAB-Map** and a 2D LiDAR, then **AMCL** for localisation on a saved map.

## What you'll do

| Task | Suggested time |
|---|---:|
| 1. Gazebo setup | 10 min |
| 2. Viewing LiDAR scans | 10 min |
| 3. RTAB-Map SLAM — mapping and exploration | 30 min |
| 4. AMCL — export the map and localise against it | 25 min |
| Physical robot — repeat the mapping exercise | 25 min |
| Spare time / Section 5 extensions | 20 min |

**Complete Sections 1–4, then the [physical-robot task](#physical-robot--repeat-the-mapping-exercise).** Section 5 and the explicitly marked extra experiments are optional. This lab is about **mapping, localisation and manual exploration**, not autonomous navigation.

The Makefile provides short launch commands; it does not do the observations or exploration for you. `make help` lists the commands and `make -n mapping` shows the ROS command behind that shortcut. You do not need to build a ROS package.

---

## 1. Gazebo setup

**Gazebo** simulates the TurtleBot3 Burger, its LiDAR and the arena. From ROS 2, simulated sensors use the same message types as the physical robot.

### 1.1 How the nodes communicate

~~~mermaid
flowchart TD
    T["Keyboard teleop"] -->|"cmd_vel"| G["Gazebo / TurtleBot3"]
    G -->|"scan and odometry TF"| R["RTAB-Map"]
    G -->|"scan and TF"| V["RViz"]
    R -->|"/map"| V
    R -->|"/map"| F["Frontier Explorer"]
    F -->|"/frontiers"| V
~~~

Teleop requests movement. Gazebo publishes scans, odometry and robot transforms. RTAB-Map uses scans and transforms to build `/map`. **RViz displays data; it doesn't create the map.**

Each robot has a name. For `ROBOT_NAME=ada`, its topics start with `/tb_ada/` and its frames include `tb_ada/odom` and `tb_ada/base_footprint`. The map and frontier topics remain `/map` and `/frontiers`. Use the **same assigned robot name in every terminal**.

### 1.2 Set up ROS 2 on the miniPC

Use your group's prepared **miniPC**. Keep the physical TurtleBot3 disconnected during simulation.

Your `ROS_DOMAIN_ID` should already be set from Lab 1: **30 + your group number**. Group 5, for example, uses `35`. Open **Terminal A**:

~~~bash
source /opt/ros/humble/setup.bash \
&& source ~/turtlebot3_ws/install/local_setup.bash \
&& export TURTLEBOT3_MODEL=burger \
&& export ROS_LOCALHOST_ONLY=1 \
&& export USE_SIM_TIME=true \
&& export ROBOT_NAME=ada \
&& echo "Robot: tb_${ROBOT_NAME} | ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
~~~

**Expected:** Your robot name and assigned group domain. If the domain is missing or wrong, set your assigned value as in Lab 1; do not use another group's domain. If a setup file or command is missing, ask a demonstrator rather than installing packages.

`ROS_LOCALHOST_ONLY=1` keeps simulation traffic on this miniPC. **Run this setup once in every new simulation terminal**, using the same robot name.

We'll use five terminals: **A** Gazebo, **B** teleop, **C** LiDAR checks and RViz, **D** RTAB-Map or AMCL, and **E** checks, frontiers and map saving. Keep long-running nodes open until told to stop them.

### 1.3 Find your Lab 5 files

If the Lab 5 files are not already in your team's workspace, get them from this repository as in Lab 4: **Code → Download ZIP**, extract it, and copy the `5/` folder into your team's repository as `labs/5`. From the repository root:

~~~bash
cd labs/5
pwd
make help
~~~

**Expected:** `pwd` ends in `/labs/5`, and `make help` lists the commands. Use this folder in every terminal.

### 1.4 Start Gazebo

**Terminal A:**

~~~bash
make sim
~~~

**Expected:** Gazebo opens with one TurtleBot3 Burger in the arena. Leave it running. If the robot does not appear, stop here and use the troubleshooting table.

### 1.5 Find the ROS topics

Open **Terminal B**, prepare it using Sections 1.2–1.3, then run:

~~~bash
ros2 topic list
ros2 topic info "/tb_${ROBOT_NAME}/cmd_vel"
~~~

| Topic (example name: `ada`) | Message or purpose |
|---|---|
| `/tb_ada/cmd_vel` | `geometry_msgs/msg/Twist` — velocity command |
| `/tb_ada/odom` | `nav_msgs/msg/Odometry` — estimated movement |
| `/tb_ada/scan` | `sensor_msgs/msg/LaserScan` — LiDAR ranges |
| `/tf` and `/tf_static` | Robot coordinate transforms |
| `/clock` | Gazebo simulation time |

**Expected:** The command topic has type `geometry_msgs/msg/Twist` and a subscriber. Check that the scan and odometry topics exist.

**Question:** Which topic commands movement? Which reports an estimate of where the robot moved?

### 1.6 Drive the simulated robot

**Terminal B:**

~~~bash
make teleop
~~~

Use the keys shown by teleop. Drive a short distance, turn and stop. **Click the teleop terminal before pressing movement keys.**

Open and prepare **Terminal C**, then inspect the position:

~~~bash
ros2 topic echo "/tb_${ROBOT_NAME}/odom" --once \
  --qos-reliability best_effort --field pose.pose.position
~~~

Move again, stop and repeat. The position values should change. As in Lab 4, `cmd_vel` requests movement; `odom` reports estimated motion.

---

## 2. Viewing LiDAR scans

A `LaserScan` contains distances measured around the robot at one instant. It is **not yet a map**.

### 2.1 Read a LiDAR scan

Keep Gazebo running. In **Terminal C**:

~~~bash
ros2 topic echo "/tb_${ROBOT_NAME}/scan" --once \
  --qos-reliability best_effort --field ranges
~~~

**Expected:** A list of distances in metres. `inf` means no valid return in that direction within the sensor's reporting range. `best_effort` matches the LiDAR's QoS.

As in Lab 3, measurements come from the `tb_<name>/base_scan` frame fixed to the robot.

### 2.2 View the scan in RViz

**Terminal C:**

~~~bash
make rviz
~~~

The supplied view starts with the live scan and robot. Inspect its settings:

1. **Global Options → Fixed Frame:** `tb_<name>/odom`.
2. **Scan → Topic:** `/tb_<name>/scan`.
3. **Scan → Reliability Policy:** **Best Effort**.
4. Enable **TF** to display the coordinate frames.

**Expected:** Scan status **OK**, with points marking surfaces around the robot. RViz uses Gazebo's `/clock`.

Rotate slowly with teleop. Compare the points with the walls in Gazebo.

**Questions:** What changes when the robot turns? Why can't one scan show the whole arena?

---

## 3. RTAB-Map SLAM — mapping

LiDAR shows what the robot can see **now**. SLAM combines scans taken at different positions to estimate a map and the robot's pose. RTAB-Map uses odometry to follow motion between scans and may reduce accumulated error when revisiting known areas.

### 3.1 From scans and TF to an occupancy grid

The **`tb_<name>/odom → tb_<name>/base_footprint`** transform tracks movement. RTAB-Map provides **`map → tb_<name>/odom`**, allowing RViz to place the robot on the accumulated map.

The `/map` topic contains a 2D **occupancy grid**: free, occupied and unknown cells.

### 3.2 Start RTAB-Map

Keep Gazebo, teleop and RViz running. Open and prepare **Terminal D**:

~~~bash
make mapping
~~~

**Expected:** RTAB-Map processes scans and prints a unique `.db` path under `~/lab5_maps/`. Each launch starts a fresh session without deleting earlier databases.

The database records the SLAM session. It is **not** the saved map image: you still need to export PGM and YAML in Section 4.

### 3.3 Check the map topic

Open and prepare **Terminal E**:

~~~bash
ros2 topic info /map
~~~

**Expected:** Type `nav_msgs/msg/OccupancyGrid` with a publisher. If no publisher appears, check Terminal D and the scan/TF inputs.

### 3.4 Show the occupancy grid in RViz

In **RViz**:

1. Change **Fixed Frame** to `map`.
2. Enable the supplied **Map** display; its topic is `/map`.
3. Check **Reliable / Transient Local** QoS for Map. Leave Scan on **Best Effort**.

**Expected:** The map appears alongside the live scan. Light cells indicate **free space**, dark cells **obstacles**, and grey cells **unknown space**.

Expand Map's properties and find **Resolution**, **Width** and **Height**. A resolution of `0.05` m/cell means **5 cm per cell**.

### 3.5 Explore and measure the map

Before moving, read the metadata in **Terminal E**:

~~~bash
ros2 topic echo /map --once --qos-reliability reliable \
  --qos-durability transient_local --field info
~~~

Calculate `map width (m) = width (cells) × resolution (m/cell)`, and likewise for height.

Drive slowly in **B**, turn to see around obstacles and stop. Run the metadata command again. **Leave some accessible unknown space for the frontier exercise.**

| Measurement | Before exploring | After exploring |
|---|---|---|
| Width × height (cells) | | |
| Resolution (m/cell) | | |
| Map width × height (m) | | |

The map's **rectangular dimensions may stay the same** while grey cells become known. Check the numbers and the colours.

**Questions:** Why is `/map` different from a single scan? How could odometry drift distort the map?

### 3.6 Explore using frontiers

A **frontier** is a boundary between known free space and unknown space. Another scan might reveal new space there, but a frontier is **not a safe route or a navigation goal**.

The working node [`exercises/frontier_explorer.py`](exercises/frontier_explorer.py) reads `/map` and publishes frontier cells on `/frontiers`. **It never moves the robot.** Open `find_frontiers()`: which cells qualify, and why must a free cell border an unknown one? There are no TODOs to complete.

**Terminal E:**

~~~bash
make frontiers
~~~

In RViz, enable **Frontiers**. Check topic `/frontiers` and **Reliable / Transient Local** QoS.

**Expected:** Frontier cells appear; Terminal E periodically reports the number of regions and approximate positions. The largest regions are listed first, not necessarily the closest or easiest.

Choose an accessible region. **Before moving, predict** whether the region count will increase, decrease or stay the same. Drive towards it manually, stop, and compare.

| Observation | Your result |
|---|---|
| Regions before → after | |
| Region chosen and why | |
| Prediction and explanation of the result | |

A scan can remove a frontier by observing unknown cells, or reveal new boundaries and create more.

### 3.7 Extra: search towards a selected area

Keep Frontier Explorer running. In RViz, with Fixed Frame `map`:

1. Choose **Publish Point**, not **2D Pose Estimate**.
2. Click a **grey unknown cell inside the map grid**, near accessible space.
3. Enable **Suggested Frontier**, topic `/suggested_frontier`.

**Expected:** One frontier cell is highlighted. Terminal E reports the clicked point, suggestion and straight-line distance **between them**—not the distance the robot must drive.

Open `closest_frontier()`. Does it use the robot's position or check for walls? Plan a safe approach, drive manually and watch the map update. When the selected point becomes known, the highlight clears.

**Question:** Why might your actual route be longer than a straight line? What extra information would autonomous navigation need?

An empty Grid Cells warning is normal when no suggestion is active. If all accessible space is mapped, skip this extra rather than restarting the whole exercise.

### 3.8 Tunable configuration

Open [`config/rtabmap_tuning.yaml`](config/rtabmap_tuning.yaml). It exposes four settings:

| Setting | What changes |
|---|---|
| `Rtabmap/DetectionRate` | Maximum processing updates per second |
| `RGBD/LinearUpdate` | Translation threshold for adding mapping nodes |
| `RGBD/AngularUpdate` | Rotation threshold for adding mapping nodes |
| `Grid/CellSize` | Occupancy-grid resolution |

Check an actual running value in E **after stopping Frontier Explorer with Ctrl+C**:

~~~bash
ros2 param get /rtabmap Rtabmap/DetectionRate
ros2 param get /rtabmap Grid/CellSize
~~~

**Predict:** What would doubling the cell size do to detail and cell count? Would a higher update rate make the sensor itself more accurate?

Keep the defaults for your first map. To experiment later, save it first, stop D, change **one** YAML value and restart `make mapping`. The launch checks the values; edits take effect on restart. Restore the original value afterwards. Spatial proximity matching can also be toggled with `make mapping SPATIAL_PROXIMITY=false`.

---

## 4. AMCL — localisation against a saved map

So far, you've built a map **online**. Now export it and use that saved, offline map with **AMCL**. AMCL estimates the robot's position from live LiDAR and odometry, but **does not extend the saved map**.

### 4.1 Save the occupancy grid

Stop the robot with teleop. Keep **Gazebo and RTAB-Map** running; stop Frontier Explorer in E if necessary.

**Terminal E:**

~~~bash
make save-map MAP=maps/sim_map.yaml
ls maps/sim_map.*
~~~

**Expected:** `Map saved successfully`, with `sim_map.pgm` and `sim_map.yaml`. The wrapper runs Nav2's map saver and refuses to replace an existing map; use `sim_map_v2.yaml` for another version.

The **PGM** contains the grid image. The **YAML** stores resolution, origin and image filename. Keep both together; they are separate from the RTAB-Map database.

### 4.2 Inspect the saved files

~~~bash
xdg-open maps/sim_map.pgm
cat maps/sim_map.yaml
~~~

Compare the image with RViz: **white** means free, **black** occupied and **grey** unknown.

| YAML field | Meaning |
|---|---|
| `image` | Matching image filename |
| `resolution` | Metres per pixel |
| `origin` | Image origin's position and orientation in the map |
| `free_thresh` / `occupied_thresh` | Pixel classification thresholds |

**Record:** width × height, resolution, approximate width in metres, and one area still unknown or difficult to map. **Why is the YAML needed as well as the image?**

### 4.3 Switch from mapping to localisation

**Stop RTAB-Map with Ctrl+C in D.** Leave Gazebo, teleop and RViz running. RTAB-Map and AMCL must not both publish the same `map → odom` transform.

**Terminal D:**

~~~bash
make localise MAP=maps/sim_map.yaml
~~~

This starts Map Server, AMCL and their lifecycle manager—not an autonomous navigation stack.

In **E**, check that AMCL is active:

~~~bash
ros2 lifecycle get "/tb_${ROBOT_NAME}/amcl"
~~~

**Expected:** `active`. Keep Fixed Frame `map` in RViz. Disable **Frontiers** and **Suggested Frontier**; enable **AMCL Particles** and **AMCL Pose**.

### 4.4 Set the initial pose and drive

In RViz, choose **2D Pose Estimate**. Click the robot's approximate position on the saved map and drag an arrow in its heading direction, using Gazebo as a visual guide.

**Expected:** A particle cloud appears. These particles represent possible robot poses. Drive slowly, turn and revisit a mapped area; check whether the live scan aligns with the saved walls.

In **E**:

~~~bash
ros2 topic echo "/tb_${ROBOT_NAME}/amcl_pose" --once \
  --qos-reliability reliable --qos-durability transient_local \
  --field pose.pose.position
~~~

Compare:

| Observation | RTAB-Map mapping | AMCL on the saved map |
|---|---|---|
| Does the map grow as you explore? | | |
| What indicates the estimated position? | | |
| What happens near a previously unseen area? | | |

**Question:** What can a localiser do with a good map that it cannot do with a poor or incomplete one?

### 4.5 Try one AMCL setting

Inspect [`config/amcl_tuning.yaml`](config/amcl_tuning.yaml): particle limits, movement thresholds and laser beams used per update. **Predict one effect** of fewer particles or fewer beams.

If time permits, stop the robot and D, change one value, restart `make localise MAP=maps/sim_map.yaml`, and set **2D Pose Estimate** again. Try a similar route and restore the setting afterwards. A narrower particle cloud is not, by itself, proof of a more accurate pose.

**Next — Physical Robot Exercise**

Once you have completed Sections 1–4 in simulation, inform your demonstrator that your group is ready. Move to your assigned robot station and wait until the demonstrator confirms that the TurtleBot3 is configured and safe to operate.

Repeat the **RTAB-Map mapping exercise** on the real robot, following the [Physical Robot instructions](#physical-robot--repeat-the-mapping-exercise) below:

1. Check the real LiDAR, odometry and transforms.
2. Start RTAB-Map and view the map in RViz.
3. Explore the arena manually with teleop.
4. Save the occupancy-grid map.
5. Compare it with your simulated map.

Do **not** repeat Gazebo setup or every earlier exercise. Physical-robot AMCL is **optional**, if time permits and the demonstrator approves.

---

## 5. Extensions

Choose one investigation; these are **not required to finish the two-hour core lab**. Ground-truth exercises are simulation-only.

### 5.1 Record one journey against ground truth

A map can look convincing even when its pose estimate is wrong. The supplied simulator publishes its reference pose on `/tb_<name>/mocap/odom`. Normal mapping does **not** use that reference.

Use the simulation setup from Section 1. If AMCL is still running in D, stop it, then start `make mapping` and disable the AMCL displays in RViz. Keep Gazebo and teleop running, and stop the robot. If you are returning from hardware, stop that session and reopen simulation terminals first.

**Terminal E:**

~~~bash
make record
~~~

This records Gazebo truth, wheel odometry and RTAB-Map estimates in a **new** folder under `output/runs/`. Follow the prompts:

| Recorder prompt | Your action |
|---|---|
| `WAITING` / `KEEP STILL` | Stay stationary while inputs arrive and frames are aligned |
| `DRIVE` | Drive slowly, turn and revisit a scanned area |
| `STOP` | Stop and remain still until `CAPTURE COMPLETE` |

The recording takes about two minutes of simulation time. Do not interrupt it or restart Gazebo. Copy and run the exact comparison command printed at the end; it points to your new run folder. You can also use `make compare RUN=<the-printed-folder>`.

A failed capture or comparison is not a usable accuracy result: read the error and repeat the capture.

### 5.2 Understand error and reported uncertainty

The evaluator fixes a frame alignment from the first eight stationary seconds and assesses the subsequent journey. It does **not** fit away errors along the whole path. Results describe **initial-pose-aligned trajectory error**, not absolute map accuracy.

**RMSE** (root mean square error) summarises the difference from ground truth. Lower centimetre RMSE means closer estimates **for this run**, not guaranteed real-world accuracy.

| Copy from your result | Value |
|---|---|
| Odometry position RMSE (cm) | |
| RTAB-Map position RMSE (cm) | |
| One observation that might explain the difference | |

**Covariance** is the estimator's *reported uncertainty*, not its measured error. In `estimator.csv`, `cxx` and `cyy` are position variances (m²); `cyaw` is heading variance (rad²).

To watch reported uncertainty while driving, use E after recording:

~~~bash
make covariance
~~~

For AMCL instead: `make covariance TOPIC=/tb_${ROBOT_NAME}/amcl_pose`.

**Ubuntu window-focus workaround:** If the covariance chart repeatedly takes focus and prevents switching to other windows, close the chart and rerun with Matplotlib's Tk backend:

~~~bash
MPLBACKEND=TkAgg make covariance
~~~

This workaround was verified in an Ubuntu student-style test and requires Tk support to be installed. Keep `make covariance` as the default command.

A descriptive position-uncertainty scale is `sqrt(cxx + cyy)` metres. With `cxx = cyy = 0.01` m², it is about **0.14 m**—not a measured 14 cm error or a guaranteed confidence radius.

**Questions:** Did covariance increase when the observations became less useful? Did actual error increase too? Can an estimator be confidently wrong?

**Why timestamps matter:** The evaluator uses each estimator pose's own timestamp and only interpolates nearby, high-rate reference samples. Sparse interpolation across a turn can cut across the actual curved path and invent an error.

### 5.3 Can you break it?

**Simulation only.** Choose one change, predict the outcome, observe it, then attempt recovery:

| Investigation | What to try |
|---|---|
| Poor initial pose | Give AMCL a deliberately wrong position or heading, drive slowly, then correct it |
| Less information | Lower `amcl_scan_beams`, restart AMCL and compare a similar route |
| Difficult motion | Compare slow movement with quicker turns, without deliberately crashing |
| Mapping configuration | Change one RTAB-Map setting from Section 3.8, or disable spatial proximity matching |

Record **the change, symptom and recovery**. Did the scan lose alignment? Did uncertainty reflect the problem? Some experiments may not break the estimate—explain why that is useful evidence too.

Manual runs are not identical trials. Do not claim that a parameter always improves accuracy from one successful run.

### 5.4 Extra: give RTAB-Map a reference pose

This optional experiment uses simulated motion capture. Save any map you want to retain, stop RTAB-Map in **D**, and start a fresh session there:

~~~bash
make mapping USE_MOCAP=true
~~~

Drive a similar route and inspect covariance or make a new recording. Stop the covariance viewer in **E** before using that terminal for recording. The reference now **helps the estimator**, so this is reference-assisted mapping—not an independent SLAM benchmark. The recorder snapshots mapper settings and labels enabled priors; the converter uses assumed noise values for weighting, not measured sensor accuracy.

**Question:** Did reported uncertainty and measured error both change? What would you need to trust a real external tracking system?

Stop D and return to `make mapping USE_MOCAP=false` when finished. If the optional optimiser is unavailable, skip this extra and ask the demonstrator; do not install anything during the lab.

---

## Physical robot — repeat the mapping exercise

The teaching team provides robot bringup. For this exercise:

- **Required:** Map the physical arena, export the map and compare it with simulation.
- **Optional:** View frontiers or run AMCL on the saved physical map.
- **Not required:** Repeat the full simulation walkthrough, record Gazebo ground truth or try failure experiments on hardware.

1. Stop the robot through teleop. Stop all simulation processes in A–E, including RViz and any covariance viewer.
2. At your assigned robot station, ask the demonstrator to start or confirm robot bringup and safe-movement instructions. Wait for their go-ahead; use the robot name they provide.
3. In each new miniPC terminal, source ROS and the workspace as in Section 1.2, keep the assigned `ROS_DOMAIN_ID`, then use:

~~~bash
export ROBOT_NAME=ada    # replace with the physical robot's assigned name
export TURTLEBOT3_MODEL=burger
export ROS_LOCALHOST_ONLY=0
export USE_SIM_TIME=false
~~~

Enter your Lab 5 folder. **Do not run `make sim`.** In E, check the real inputs before moving:

~~~bash
ros2 topic info "/tb_${ROBOT_NAME}/scan"
ros2 topic info "/tb_${ROBOT_NAME}/odom"
ros2 run tf2_ros tf2_echo "tb_${ROBOT_NAME}/odom" "tb_${ROBOT_NAME}/base_footprint"
~~~

**Expected:** Sensor publishers and a live transform. Stop `tf2_echo` with Ctrl+C. If the namespace or frames differ, ask the demonstrator to confirm the prepared bringup; do not guess remappings.

Start each command in its own prepared terminal:

| Terminal | Command |
|---|---|
| D — mapping | `make mapping` |
| C — RViz | `make rviz FIXED_FRAME=map` |
| B — manual driving | `make teleop` |

Enable **Map** in RViz. Drive slowly around the real arena, looking around corners and revisiting a scanned area. You can use `make frontiers` in E as in Section 3.6.

**Only one controller may send movement commands.** Keep the robot on the floor, maintain clearance and use the demonstrator's stop procedure. Do not run the failure experiments on hardware.

Stop the robot and Frontier Explorer, but leave mapping running. In E:

~~~bash
make save-map MAP=maps/real_map.yaml
xdg-open maps/real_map.pgm
~~~

Keep both `real_map.pgm` and `real_map.yaml`. **Compare:** which walls or corners are less clean than in simulation? Which spaces stayed unknown, and why? Gazebo ground truth is unavailable here.

When finished, stop teleop and mapping and follow the demonstrator's shutdown procedure. **Optional, with demonstrator approval:** repeat AMCL from Section 4 using `MAP=maps/real_map.yaml`; stop RTAB-Map first.

---

## If something doesn't work

Check **which node, topic or display** failed. Do not change the miniPC's ROS installation or another group's settings.

| Problem | Check |
|---|---|
| `make` cannot find a target | Return to the Lab 5 folder containing `Makefile` |
| Gazebo or robot missing | Read A's error; check `ros2 service list` for `/spawn_entity` and ask a demonstrator; do not spawn a second robot blindly |
| Scan/odometry missing | Gazebo or robot bringup running? Same robot name, domain and localhost setting in all terminals? |
| Scan absent in RViz | Correct namespaced scan topic, Best Effort QoS, odom fixed frame before SLAM, correct time setting |
| RTAB-Map exits or map missing | Read D; check `ros2 node list`, scan and TF inputs |
| Map blank in RViz | Enable Map, fixed frame `map`, Reliable / Transient Local QoS |
| Frontiers/suggestion absent | Enable the displays; click unknown space inside the map grid. Empty suggestions are normal |
| Map saver fails | Keep mapper running. Existing files are protected: choose a new map name |
| AMCL has no pose | Confirm lifecycle `active`, set 2D Pose Estimate, then move slowly |
| Robot jumps between poses | Confirm RTAB-Map and AMCL are not both running |
| Recording fails | Check simulation truth/odometry/estimate topics, keep still initially and follow the printed diagnostic |

When asking for help, show the **command, error and check you tried**.

## Before you finish

- [ ] Observed a live scan and explained why it is not a map
- [ ] Built a map, explored a frontier and saved PGM + YAML
- [ ] Used AMCL to localise against that saved map
- [ ] Mapped the physical arena and saved a separate real-robot map
- [ ] Explained one difference between mapping and localisation
- [ ] Optional: investigated configuration, ground truth, uncertainty or a failure
