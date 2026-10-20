> **Background only.** This is the original Lab 3 sheet, kept for reference. For the lab session, follow the [current Lab 3](../README.md). Do not run the controller in this folder on a robot.

# Lab 3 — TF2, Bringup, Sensors and Odometry

Lab 1 covered bringup and teleop at a glance. This lab goes underneath both: how ROS 2 tracks the position of every part of the robot with a tree of coordinate frames, what the sensor topics actually carry, and — through a controlled experiment — why odometry alone can't be trusted as ground truth. One data pipeline runs through all of it: wheels and LiDAR, through `/odom`, `/tf` and `/scan`, to what you can read on the command line and see in RViz.

This is a **physical-robot lab** — do Sections 1–5 on the real TurtleBot3 in the session. A Gazebo version of the same exercises is provided in Section 6, for extra practice from home if you want it.

> Sections marked **TODO** are still being finalised.

---

## The goal for this lab session

- [ ] Bringup nodes listed, with one sentence on each one's role
- [ ] `frames.pdf` generated, and a static and a dynamic transform both echoed
- [ ] `/scan` and `/odom` confirmed healthy with `topic hz`, and a raw message read from each
- [ ] RViz laser scan matched against a real object placed in front of the robot
- [ ] A square driven and the drift measured; the physical drift demo completed

By the end of the lab you should be able to:

1. Explain the TF2 frame tree and identify the key frames on the TurtleBot3 Burger.
2. List the nodes bringup starts and explain what each one does.
3. Use `view_frames`, `tf2_echo` and RViz to inspect transforms.
4. Confirm a sensor is publishing at the expected rate, and read a raw message from it.
5. Read `/odom` data and demonstrate, physically, why odometry drifts — and what SLAM adds on top.

---

## 1. Prepare the robot

These are the same preparation steps you'll use in every physical-robot lab from here on.

### 1.1 Source your environment

Make sure ROS 2 is sourced in every terminal you open — see [Lab 2, Section 1.1](../../2/README.md#11-make-sure-your-environment-is-active) if you need the reminder. Pixi is not a supported route from Lab 2 onward — refer to the [Pixi documentation](https://pixi.sh/latest/) if you need help.

### 1.2 Set your ROS_DOMAIN_ID

```bash
echo $ROS_DOMAIN_ID
```

This must match your group's assigned ROS domain ID (30 plus the group number), and the TurtleBot must use the same domain ID. It should already be set from [Lab 1, Section 4.2](../../1/README.md#42-set-your-ros_domain_id) — if it's blank, set it permanently:

```bash
echo 'export ROS_DOMAIN_ID=<your assigned ROS domain ID> #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc
```

(The first line makes it permanent by adding it to your shell's startup file; the second applies it to this terminal now instead of waiting for a new one.)

### 1.3 SSH in and bring up the robot

Same as Lab 1: SSH into the robot, then bring it up in that session.

```bash
ssh ubuntu@<TB3_IP>
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

Leave that terminal running for the rest of the lab. See [Lab 1, Sections 5.2–5.3](../../1/README.md#52-connect-to-the-robot-with-ssh) for what healthy output looks like and what to do if SSH won't connect.

### 1.4 Verify the robot's topics are visible

Back on your own machine:

```bash
ros2 topic list
```

You'll see plenty of topics; this lab is built around three of them, so confirm they're all there:

```
/tf
/odom
/scan
```

If any are missing, SSH back in and restart bringup (`ros2 launch turtlebot3_bringup robot.launch.py`).

Don't move on until all four checks above pass.

---

## 2. TF2 and bringup

### 2.1 What does bringup actually start?

`robot.launch.py` doesn't start one program — it starts several nodes that together make the robot usable. Find them:

```bash
ros2 node list
```

You should see `turtlebot3_node` and `robot_state_publisher` among them. Inspect two of them:

```bash
ros2 node info /turtlebot3_node
ros2 node info /robot_state_publisher
```

Look at the **Publishers** and **Subscribers** sections of each, and write one sentence per node describing its role based on what it publishes and subscribes to — for example, "`turtlebot3_node` subscribes to `/cmd_vel` and publishes `/odom`, `/tf`, and `/joint_states`."

### 2.2 The TurtleBot3 Burger frame tree

The transforms you'll inspect below come from the bringup nodes you just listed. The frames form a tree:

```
odom
 └── base_footprint
      └── base_link
           ├── base_scan
           ├── wheel_left_link
           ├── wheel_right_link
           └── imu_link
```

Five things worth understanding about it:

1. **Each frame is a set of coordinate axes** fixed to one physical thing — the robot body, the LiDAR, a wheel.
2. **A transform is the edge between frames** — the translation and rotation from parent to child.
3. **Static transforms** (on `/tf_static`) never change: `base_link → base_scan` is fixed because the LiDAR is bolted on.
4. **Dynamic transforms** (on `/tf`) update continuously: `odom → base_footprint` changes every time the wheels turn.
5. **Every transform is timestamped**, so you can ask "where was `base_scan` relative to `odom` at time T?" — important when a laser scan and an odometry reading aren't perfectly synchronised.

The `map` frame doesn't exist yet — you won't see it until SLAM (Lab 5) adds a `map → odom` transform that corrects for drift. Until then, `odom` is the root of the tree, and it drifts, which is exactly what Section 4 is about.

### 2.3 Identify static transforms

```bash
ros2 topic echo /tf_static --once
```

Note two static transforms: parent frame, child frame, and approximate translation (x, y, z).

### 2.4 Generate the TF tree diagram

```bash
ros2 run tf2_tools view_frames
```

This creates `frames.pdf` in your current directory — open it. TF should be flowing at roughly 20 Hz; if the PDF comes out empty, run `ros2 topic hz /tf` — 0 Hz means bringup isn't publishing transforms.

### 2.5 Echo a static transform: base_link → base_scan

```bash
ros2 run tf2_ros tf2_echo base_link base_scan
```

```
At time 0.0
- Translation: [x: -0.032, y: 0.000, z: 0.172]
- Rotation: in Quaternion [x: 0.000, y: 0.000, z: 0.000, w: 1.000]
```

These values are fixed — the LiDAR doesn't move relative to the robot body. On the Burger it sits about 3.2 cm behind and 17.2 cm above `base_link`.

### 2.6 Echo a dynamic transform: odom → base_footprint

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

Now drive the robot from another terminal:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

The translation values in the `tf2_echo` terminal change as the robot moves — `x` and `y` grow as you drive forward and turn. Ctrl+C the teleop terminal when done.

---

## 3. Sensors and RViz

The Burger has three main sensor sources: a 2D LiDAR, an IMU, and wheel encoders (read via `/odom` — the subject of Section 4). Here you'll confirm each is active, read a raw message, and visualise the LiDAR in RViz.

### 3.1 List sensor topics and check publish rates

```bash
ros2 topic list
```

On the Burger you should find at minimum:

| Topic | Sensor |
|---|---|
| `/scan` | 2D LiDAR (LDS-02) |
| `/imu` | IMU (on the OpenCR board) |
| `/battery_state` | Battery level |
| `/joint_states` | Wheel encoder positions |

A healthy bringup publishes at predictable rates. `ros2 topic hz` is the fastest first diagnostic — if `/scan` shows 0 Hz, the LiDAR isn't running at all, not just publishing slowly. Check the two most important ones (Ctrl+C to exit each):

```bash
ros2 topic hz /scan
ros2 topic hz /odom
```

Both should be publishing at roughly the expected rate; if either is 0 Hz, restart bringup before continuing. Then do the same for `/imu` and `/joint_states` and note whether they're faster or slower than `/scan`.

### 3.2 Read a raw sensor message

```bash
ros2 topic echo /scan --once
```

The key fields in a `sensor_msgs/LaserScan`:

```yaml
header:
  frame_id: "base_scan"       # data is in this frame
angle_min                     # start angle of sweep (radians)
angle_max                     # end angle of sweep (radians)
angle_increment               # angular step between readings
range_min                     # minimum valid range (m)
range_max                     # maximum valid range (m)
ranges: [0.45, 0.46, ..., inf, inf]  # 360 distance readings
```

Work out how many elements are in `ranges` from `angle_min`, `angle_max` and `angle_increment`, and note what value shows up for directions where nothing is in range — and why that's the right way to represent it.

Now the IMU:

```bash
ros2 topic echo /imu --once
```

Tilt the robot by hand and echo again — `linear_acceleration` and `angular_velocity` should change.

### 3.3 Visualise frames and the laser scan in RViz

On your own machine:

```bash
rviz2
```

1. Set **Fixed Frame** to `odom` (top of the Displays panel).
2. **Add** → **TF** → OK.
3. **Add** → **LaserScan** → set Topic to `/scan` → OK.
4. Drive the robot with teleop and watch.

Take a screenshot showing the TF axes and laser scan points, and write two or three sentences on why the laser points move with the robot, and what the axes on each frame represent.

Now switch **Fixed Frame** to `base_link` and put an object (a bag, a book) about 0.5 m in front of the robot — the cluster of scan points should match its position and rough shape.

With `odom` as the fixed frame you see the robot move through the world; with `base_link` the robot stays centred and the scan rotates around it instead, which makes it easier to line scan geometry up with the room. Same data, two views.

---

## 4. The odometry experiment

Odometry integrates wheel encoder counts to estimate position. Every small encoder tick contributes a tiny position update, and errors — wheel slip, an uneven floor, calibration — accumulate, so the estimate drifts away from the truth over time.

### 4.1 Read a single odometry message

```bash
ros2 topic echo /odom --once
```

```yaml
header:
  frame_id: "odom"             # reference frame for this data
child_frame_id: "base_footprint"
pose:
  pose:
    position:
      x: 0.0                   # metres east of start
      y: 0.0                   # metres north of start
      z: 0.0                   # always ~0 for ground robot
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0                   # encodes heading (with w)
      w: 1.0                   # encodes heading (with z)
twist:
  twist:
    linear:
      x: 0.0                   # forward velocity (m/s)
    angular:
      z: 0.0                   # turning rate (rad/s)
```

Drop `--once` to watch it stream live instead of a single snapshot.

### 4.2 Reading heading from the quaternion

On flat ground, only `z` and `w` matter:

| Heading (compass) | `orientation.z` | `orientation.w` |
|---|---|---|
| 0° — facing forward at start | 0.000 | 1.000 |
| 90° — turned left | 0.707 | 0.707 |
| 180° — facing backward | 1.000 | 0.000 |
| −90° — turned right | −0.707 | 0.707 |

`yaw_degrees = 2 × atan2(z, w) × (180/π)`

### 4.3 Watch the pipeline respond to teleop

Before the drift experiment, watch how a single key press propagates through the system. You'll need three terminals, each with ROS 2 sourced.

**Terminal 1** — teleop:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Terminal 2** — the velocity commands it's sending:

```bash
ros2 topic echo /cmd_vel
```

Press the forward key in Terminal 1 and watch Terminal 2:

```yaml
linear:
  x: 0.22   # m/s forward
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0    # rad/s — non-zero when turning
```

Note the values for forward, left, and releasing all keys.

**Terminal 3** — `/odom` responding:

```bash
ros2 topic echo /odom
```

Drive slowly forward about 0.5 m, then stop. `pose.position.x` should increase while moving, `twist.linear.x` should roughly match what you saw in `/cmd_vel`, and velocity should return to ~0 when you release the key.

Now swap the `/odom` echo in Terminal 3 for:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

Drive again — you'll see the same position change as `/odom` did, because the TF tree and `/odom` are driven by the same source: the wheel encoders inside `turtlebot3_node`. Note what's the same and what (if anything) differs in how the two present the data.

### 4.4 Drive a square and measure drift

Use the official TurtleBot3 "Relative Move" example — see the [ROBOTIS docs](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/) and the [source on GitHub](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble/turtlebot3_example/turtlebot3_example) for more.

1. Mark the robot's starting position on the floor with tape.
2. Record the starting odometry (`ros2 topic echo /odom --once`) — note `position.x`, `position.y`, `orientation.z/w`.
3. With bringup still running, open a new terminal and start the example — this is how the docs say to run it:

   ```bash
   ros2 run turtlebot3_example turtlebot3_relative_move
   ```
4. Enter `x: 1`, `y: 0`, `theta: 90` four times to drive a square.
5. Record the odometry again after the fourth move.
6. Compare the robot's actual final position against the tape.

| Measurement | Start | End |
|---|---|---|
| `position.x` | | |
| `position.y` | | |
| Heading (°) | | |

### 4.5 Physical drift demonstration

This makes drift obvious by deliberately reducing traction on one wheel.

1. Rest one wheel on a thin book or folded paper (~5 mm), so it has slightly less traction than the other.
2. Watch `/odom` in a terminal.
3. Drive forward slowly for 1 m with teleop, then stop.
4. Note the heading (`orientation.z`, `orientation.w`) — the robot should have drifted left or right of straight.
5. Remove the paper and pick the robot up to put it back at the start. Note that this only moves the robot — `/odom` keeps accumulating from wherever it was, drift included, since nothing has reset it in software (`turtlebot3_node` does expose a reset service, which you can look up).

Did the robot travel in a straight line? Did the odometry report a straight line? What does that tell you about the relationship between encoder counts and real-world motion?

Odometry trusts encoder counts unconditionally. One wheel turning faster — traction, a bump, a slope — produces drift the robot has no way to detect from encoders alone. This is the core argument for SLAM.

### 4.6 Think about

1. **Drift magnitude:** the Euclidean distance between your start and end odometry positions from Section 4.4: `√((x_end − x_start)² + (y_end − y_start)²)`. What did you get?
2. **Drift direction:** ahead, behind, left, or right of the start? What might cause that?
3. **Which segment caused the most error** — the straight sections or the turns? Why?
4. **Why does SLAM add `map → odom` rather than correcting the `/odom` estimate directly?** Think about what happens to code that was already listening to `/odom` before and after a correction.

---

## 5. Extension: verify the transform chain

This reinforces the quaternion table from Section 4.2: predict the transform odometry should report after a known move, then check it live.

Before moving, capture a baseline of the whole tree:

```bash
ros2 run tf2_tools view_frames
```

Then start streaming the live transform, and leave it running in its own terminal for the rest of this exercise:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

`odom` is fixed — a stake in the ground at startup — while `base_footprint` moves with the robot, so this answers "where is the robot relative to where it started?" That's exactly why drift shows up here: after a full square the maths is internally consistent, but the reported pose no longer matches the robot's true physical position.

**Predict before you move.** Before issuing `x: 1`, `y: 0`, `theta: 90` in `turtlebot3_relative_move`, write down what `tf2_echo` should report afterwards: a translation of `(1.0, 0.0, 0.0)` — 1 m along the starting x-axis — and a 90° rotation about Z, which as a quaternion is `(x: 0, y: 0, z: 0.707, w: 0.707)` (from the table in Section 4.2). Run the move and check your prediction.

**Chain a second move.** Predict the resulting transform in the `odom` frame after a further `x: 1`, `y: 0`, `theta: 0`. Each move compounds onto the previous one — the new pose is `T_new = T_current · ΔT`, where `ΔT` is the transform your input describes. For any 2D rotation by angle θ, the quaternion components are `z = sin(θ/2)` and `w = cos(θ/2)` (which is where 0.707 came from — sin and cos of 45°). The robot is now facing 90°, so "forward" in its own frame is the **+Y direction** in the odom frame — that matters for the translation. Work it out by hand, then verify.

**Inspect the tree between moves.** Run `view_frames` again after each one. Does the tree's structure change, or only the values on the `odom → base_footprint` edge?

For each move, record the predicted and observed translation and quaternion, compute the prediction error, and write a sentence on any discrepancy.

---

## 6. Extra practice at home (Gazebo)

Everything in Sections 1–5 runs the same way in Gazebo, if you want to go over it again outside the session:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

All the TF, sensor and odometry topics are identical. The odometry experiment still shows drift, just less of it, since simulation has no wheel slip — and the book-under-one-wheel demo can't be reproduced at all; drive aggressively through tight turns instead to see drift some other way.

This is optional. Sections 1–5 are done on the physical robot in the lab session regardless.

---

## 7. Before you leave

- [ ] Listed the bringup nodes with one sentence per node
- [ ] Identified two static transforms from `/tf_static`
- [ ] Generated `frames.pdf`
- [ ] Echoed both the static (`base_link → base_scan`) and dynamic (`odom → base_footprint`) transforms
- [ ] `/scan` and `/odom` confirmed healthy with `topic hz`; `/imu` and `/joint_states` rates recorded
- [ ] Read one raw message each from `/scan` and `/imu`
- [ ] RViz scan matched to a real object, in both `odom` and `base_link` fixed frames
- [ ] `/cmd_vel` values recorded for forward, turn, and stop
- [ ] Square driven, drift measured and the Euclidean error computed
- [ ] Physical drift demonstration completed

---

## 8. When something goes wrong

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 topic list` shows nothing from the robot | Wrong or unset `ROS_DOMAIN_ID` | `echo $ROS_DOMAIN_ID`; match it to your group number |
| `/tf` missing from the topic list | Bringup not running | SSH in and restart `robot.launch.py` |
| `ros2 topic hz /scan` shows 0 Hz | LiDAR not spinning, or bringup crashed | SSH in, check bringup is actually running, listen for the LiDAR spinning up |
| `tf2_echo` says "waiting for transform" for 10+ seconds | TF not populated yet | Wait a little longer, or check `ros2 topic hz /tf` |
| `view_frames` produces an empty PDF | TF not flowing | `ros2 topic hz /tf` should read ~20 Hz |
| `/cmd_vel` shows nothing when you press keys | Teleop terminal lost keyboard focus | Click on the teleop terminal first |
| `/odom` not updating while driving | Encoder data not publishing | Check `ros2 topic hz /odom` and `/joint_states`; restart bringup if either is 0 Hz |
| `ros2 node info` shows no publishers or subscribers | Node name typo | Re-run `ros2 node list` and copy the name exactly |
| RViz shows nothing | Wrong fixed frame | Set Fixed Frame to `odom`; confirm `/tf` exists |
| RViz scan is a ring far from the origin | Fixed Frame is `odom`, robot has moved | That's correct, not broken — switch to `base_link`, or accept the offset |
| Odometry jumps suddenly | Robot was picked up, or a wheel slipped on a seam | Expected — the estimate is purely encoder-based, so any wheel event shows up in it |

If you're stuck for more than a few minutes, ask.

---

## 9. Before Lab 4

In Lab 4 you'll write your first ROS 2 nodes in Python — a publisher that drives the robot through an open-loop square on `/cmd_vel`, and code that reads odometry — then reproduce the same square in the PyBullet simulator. Everything from this lab (which nodes exist, what they publish, why the estimate drifts) is the foundation for that.

---

## Appendix A — Command reference

| Concept | Tool / command | Purpose |
|---|---|---|
| List bringup nodes | `ros2 node list` | `turtlebot3_node`, `robot_state_publisher`, … |
| Inspect a node | `ros2 node info /<node>` | Publishers, subscribers, services |
| View the frame tree | `ros2 run tf2_tools view_frames` | Generates `frames.pdf` |
| Raw transform stream | `ros2 topic echo /tf_static --once` | Inspect transform messages |
| Echo a static transform | `ros2 run tf2_ros tf2_echo base_link base_scan` | Fixed offset, doesn't change |
| Echo a dynamic transform | `ros2 run tf2_ros tf2_echo odom base_footprint` | Changes as the robot moves |
| Check sensor health | `ros2 topic hz /scan` | ~5 Hz expected; 0 Hz means a fault |
| Raw sensor message | `ros2 topic echo /scan --once` | `ranges` array, 360 readings |
| Watch commands | `ros2 topic echo /cmd_vel` | `linear.x`, `angular.z` on each key press |
| Odometry snapshot | `ros2 topic echo /odom --once` | Position, orientation, velocity |
| Visualise everything | `rviz2` | TF axes and laser scan together |
