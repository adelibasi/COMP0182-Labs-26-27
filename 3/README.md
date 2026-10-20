# Lab 3 — TF2, Sensors, Odometry and a Robot Square

In **Lab 1**, you drove a TurtleBot3. In **Lab 2**, you used ROS topics, services and cancellable actions in turtlesim. This week, you return to the real robot: sensor topics provide measurements, TF2 connects coordinate frames, and odometry estimates movement. You'll use them to investigate a four-leg square.

**Today's question:** when we command a robot to move, how do we know where it actually went?

Use your group’s Ubuntu 22.04 / ROS 2 Humble miniPC, assigned TurtleBot3 Burger and approved floor space. **No new installation is needed.**

**Five checkpoints:** (1) ROS graph → (2) TF2 frames → (3) sensors and RViz → (4) commands versus odometry → (5) predicted versus measured square. This README contains all the tasks: **predict → observe → explain** at each checkpoint. Keep brief notes or permitted screenshots only for measurements you need to compare, especially in Sections 4–5. Do not edit shared lab files; GitHub's preview is read-only. “Go further” is optional.

| Terminal | Location | Purpose |
|---|---|---|
| **A** | SSH from miniPC to robot | Bringup; leave running |
| **B** | MiniPC | ROS inspection and measurements |
| **C** | MiniPC | RViz (Section 3), then teleop (Section 4), then Python (Section 5); close each before the next |
| **D** | MiniPC — open in Section 4.2 only | Watch TF while teleop runs |

**Start with A, B and C.** Open D only when you need a separate TF observer in Section 4.2. Source `/opt/ros/humble/setup.bash` in each new miniPC terminal; make sure its ROS domain matches your assigned robot.

**Movement safety (as in Lab 1):** clear floor, low speed, observer and known physical stop/power control. **Section 4:** familiar Lab 1 teleop, with its usual precautions. **Section 5:** the custom four-leg controller, **only after** a demonstrator confirms its manual-input safety check on your robot. Space or `s` requests a stop in teleop; **Ctrl+C exits a program, not an emergency stop**. If movement is unexpected, use the physical stop, tell staff and enter no further commands.

---

## 1. Reconnect your robot (Lab 1 recap)

Use your assigned robot and miniPC, as in [Lab 1](../1/README.md). Check the ROS setup before starting anything.

### 1.1 Terminal B — miniPC

~~~bash
source /opt/ros/humble/setup.bash
echo "$ROS_DISTRO"
echo "$ROS_DOMAIN_ID"
~~~

Expect `humble` and your group's assigned ROS domain. If the domain is wrong, ask a demonstrator. **Do not change the robot's domain.**

### 1.2 Terminal A — connect to your assigned robot

In **Terminal A**, SSH into your assigned robot, as in Lab 1. Check that the prompt shows the robot's `ubuntu@...` shell, then run **on the robot**:

~~~bash
source /opt/ros/humble/setup.bash
# Some robots use the existing workspace for the LiDAR driver.
if [ -f "$HOME/turtlebot3_ws/install/setup.bash" ]; then
    source "$HOME/turtlebot3_ws/install/setup.bash"
fi
export TURTLEBOT3_MODEL=burger
echo "$ROS_DOMAIN_ID"
echo "$LDS_MODEL"
~~~

Check that the robot and miniPC domains match. Leave the robot's LiDAR setting as configured.

### 1.3 Check bringup and inspect the ROS graph

**In Terminal B (miniPC)**, check whether the robot is already running:

~~~bash
ros2 node list
~~~

If `/turtlebot3_node` appears, **do not start bringup again**. If it does not appear but **Terminal A** already shows `Run!` launch messages, leave bringup running and ask a demonstrator to check ROS discovery.

**Only if bringup is not running and a demonstrator confirms**, start it once in **Terminal A (robot)**:

~~~bash
ros2 launch turtlebot3_bringup robot.launch.py
~~~

Keep the robot still while it calibrates. Wait for `Run!` and leave Terminal A open. If launch fails, ask for help rather than repeatedly restarting.

**Back in Terminal B**, inspect the graph:

~~~bash
ros2 node list
ros2 topic list
ros2 node info /turtlebot3_node
ros2 node info /robot_state_publisher
ros2 topic info -v /odom
~~~

Look for `/odom`, `/scan` and `/tf`. Nodes may take a few seconds to appear. If one stays missing, ask staff. **A listed topic does not prove messages are arriving.**

**Example ROS message flow** (from one teaching robot; check yours). Rectangles are nodes; ovals are topics.

~~~mermaid
flowchart TB
    cmd(["/cmd_vel"]) --> robot["turtlebot3_node"]
    robot --> imu(["/imu"])
    robot --> joints(["/joint_states"])
    imu --> drive["diff_drive_controller"]
    joints --> drive
    joints --> state["robot_state_publisher"]
    drive --> odom(["/odom"])
    drive --> tf(["/tf"])
    state --> tf
    state --> tfStatic(["/tf_static"])

    classDef node fill:#E8F7F0,stroke:#2C8C6E,color:#123A31,stroke-width:1.5px;
    classDef topic fill:#EEF2FF,stroke:#5A67C9,color:#24305D,stroke-width:1.5px;
    class robot,drive,state node;
    class cmd,imu,joints,odom,tf,tfStatic topic;
~~~

**Checkpoint 1:** From your output, identify one publisher, one subscriber and the role of each inspected node. Who publishes `/odom`? Why don't ROS topic commands need the robot's SSH IP? Discuss your answers with your group.

---

## 2. TF2 — where is the sensor?

The LiDAR measures distances in its own coordinate frame. TF2 tells ROS where that frame sits relative to the robot.

### 2.1 Find the frame tree

In **Terminal B**:

~~~bash
ros2 node info /robot_state_publisher
ros2 run tf2_tools view_frames
ls -l frames*.pdf
~~~

Open the generated PDF on the miniPC. The filename may include a timestamp. If the tool or PDF is missing, ask staff; **do not install packages on the shared miniPC**. You can still try `tf2_echo` in Section 2.2.

Compare your PDF with this typical TurtleBot3 frame tree:

~~~text
odom
 └── base_footprint
      └── base_link
           ├── base_scan          (LiDAR)
           ├── imu_link           (IMU)
           ├── wheel_left_link
           ├── wheel_right_link
           └── caster_back_link
~~~

`odom → base_footprint` gives the robot's estimated pose. The LiDAR and IMU stay fixed to `base_link`; wheel frames can rotate. A dynamic transform can keep updating even when the robot is stationary. In mapping, `map → odom` may correct global position while local `odom` remains continuous.

**Predict:** Which transform changes when the robot moves? Which sensor transform stays fixed?

### 2.2 Measure two transforms

In **Terminal B**, run the first command, read a few values, then press **Ctrl+C** to stop it:

~~~bash
ros2 run tf2_ros tf2_echo base_link base_scan
~~~

Now repeat for the robot's estimated pose:

~~~bash
ros2 run tf2_ros tf2_echo odom base_footprint
~~~

You may briefly see `Invalid frame ID` while TF2 receives the frames. If valid readings follow, the command is working. If no readings arrive, ask staff to check the frames and ROS domain.

`base_link → base_scan` is the fixed LiDAR mounting offset; `odom → base_footprint` is a time-updated pose estimate. Record the **actual** translation (metres) and rotation (degrees or radians). A dynamic transform may report almost identical values while the robot is still.

**Checkpoint 2:** Identify one static and one dynamic transform (frame names and readings). Why does the LiDAR move relative to the room while its mounting offset stays fixed? **Go further (optional):** If the LiDAR bracket moves forward 10 cm, which transform changes?

---

## 3. Sensors — connect numbers to the room

In Lab 2 you inspected message types; now read real LiDAR and IMU measurements and find the corresponding points in RViz.

### 3.1 Inspect and interpret

In **Terminal B**, check the sensor types and read one LiDAR scan:

~~~bash
ros2 topic type /scan
ros2 topic type /imu
ros2 topic info -v /scan
ros2 topic echo --once --qos-reliability best_effort /scan
~~~

The output is long. Find `header.frame_id` (sensor frame), `angle_min` and `angle_increment` (radians), and `ranges` (metres).

For `ranges[i]`, calculate the ray's direction as **`angle_min + i × angle_increment`**. The array uses a zero-based index (starting at 0); **`ranges[0]` is not necessarily straight ahead**. Angles are relative to the sensor frame. Ignore `.nan` and `.inf` readings.

**Worked example (invented values, not your robot):** With `angle_min = -1.0`, `angle_increment = 0.1` and `ranges = [0.40, 0.65, inf]`, `ranges[1]` is **0.65 m at −0.9 radians**.

**Your turn:** Choose a finite `ranges[i]` near the start of your scan. Calculate its angle from the real `angle_min` and `angle_increment`, and note the index, distance and angle.

For the IMU:

~~~bash
ros2 topic info -v /imu
ros2 topic echo --once --qos-reliability best_effort /imu
~~~

Record `angular_velocity` (x, y, z) in rad/s while the robot is still; ignore covariance arrays. Best Effort echo can receive from either Best Effort or Reliable publishers. If no message arrives, ask staff to check the domain, publisher and QoS.

### 3.2 Compare message rates

**Predict:** should the LiDAR and odometry messages arrive at the same rate? Check in **B**:

~~~bash
ros2 topic hz /scan
~~~

Wait for several averages, press Ctrl+C, then:

~~~bash
ros2 topic hz /odom
~~~

Note the final average for each topic in Hz (messages received per second, **not** robot speed). Compare the two rates: do they have to match? **No output ≠ 0 Hz**; inspect endpoints and QoS.

### 3.3 See the scan in RViz

In **Terminal C** (miniPC), check that the ROS domain matches your robot, then launch RViz:

~~~bash
source /opt/ros/humble/setup.bash
echo "$ROS_DOMAIN_ID"
rviz2
~~~

If the domain is wrong, ask staff. Leave robot bringup running in A.

**In RViz, use the Displays panel on the left:**

1. Under **Global Options**, set **Fixed Frame** to `odom`.
2. Click **Add → TF** if TF is missing. Untick any **Map** displays; they need an occupancy grid (usually `/map` from SLAM or a map server), which we are **not** creating here. Keep **Grid** if useful.
3. Click **Add → LaserScan** if it is missing. Expand **LaserScan**, set **Topic** to `/scan` and press Enter if you typed it. Do not leave Topic blank.
4. Under **LaserScan display properties → Topic**, set **Reliability Policy** to **Best Effort**. The `/scan` publisher uses Best Effort; a Reliable subscription cannot receive it.
5. Check **LaserScan → Status**. Look for **Ok** and coloured scan points. Zoom in if the points are hard to see.

**If you see an error:**
- `Error subscribing: Empty topic name` → choose **Topic = /scan**.
- `No messages` → check the topic, ROS domain and **Best Effort** setting.
- `No transform` → check **Fixed Frame = odom** and TF. Ask staff if it persists.

A brief TF timing warning may appear while data loads. If scan points stop updating or timing errors repeat, show staff; do not change the robot's clocks. Widen the Displays panel if a property value is cut off.

Match a visible scan cluster to a **stationary wall or object** in the room. Ask staff before placing a new object near the robot. **Do not move the robot for this task.** These are live LiDAR points, **not an occupancy-grid map**.

**Predict:** What would change if Fixed Frame were `base_link` rather than `odom`? Try it, then restore `odom`. The view may look nearly the same while the robot is stationary.

**Checkpoint 3:** Check your calculated ray against RViz; identify a real object matching the scan points. Recall the two message rates, Fixed Frame and LaserScan status. **Diagnose:** `/scan` messages arrive but RViz is blank. Give **two causes and a test for each**.

**Next:** save the observation and **close RViz** using Ctrl+C in **C**. You'll reuse C for teleop in Section 4; you will open D only when Section 4.2 calls for it.

---

## 4. Odometry — a request is not a measurement

Before moving, distinguish a **velocity command**, an **odometry estimate** and what you can actually see on the floor. A command does not guarantee movement, and odometry can drift.

~~~mermaid
flowchart LR
    cmd["/cmd_vel<br/>requested velocity"] --> robot["Robot<br/>motors and sensors"]
    robot --> odom["/odom<br/>estimated pose"]
    robot -.-> floor["Floor observation<br/>what you actually see"]

    classDef topic fill:#EEF2FF,stroke:#5A67C9,color:#24305D,stroke-width:1.5px;
    classDef other fill:#E8F7F0,stroke:#2C8C6E,color:#123A31,stroke-width:1.5px;
    class cmd,odom topic;
    class robot,floor other;
~~~

*Conceptual flow, not a complete ROS graph.* Here **pose** means position (x, y) and orientation (heading). The odometry estimate is relative to the `odom` frame; a floor observation is not automatically a distance measurement.

### 4.1 Record a stationary baseline

Keep the robot still. In **Terminal B**:

~~~bash
ros2 topic echo --once /odom --field pose.pose
ros2 topic info /cmd_vel
~~~

**Read the pose:** `position.x` and `position.y` are in metres. `orientation` is a **quaternion** (x, y, z, w), not an angle. For a level robot moving in a plane, calculate heading (**yaw**) as `2 × atan2(z, w)` radians; multiply by 180/π for degrees. Note your starting x, y and heading (yaw) so you can compare them after teleop. They do **not** need to start at zero; do not reset odometry.

**Check `/cmd_vel`:** `Publisher count: 0` means no command publisher was discovered at that moment; `Subscription count: 1` means one node is listening. If a publisher is unexpectedly active, ask staff before proceeding. These counts **do not prove the robot is physically safe**. Confirm it is stationary; movement precautions apply in Section 4.2.

### 4.2 Compare a movement request with the estimate — Lab 1 teleop

**Same keyboard teleop as Lab 1 §5.5**. Use a clear floor area, an observer, low speed and a ready physical stop. **No separate controller qualification is needed for this ordinary Lab 1 activity.** If movement is unavailable, record `NOT RUN` and continue without moving the robot.

**Step 1 — Terminal D: watch TF.** After closing RViz in C, open a new miniPC terminal D. A new terminal may have a different ROS domain. First check:

~~~bash
source /opt/ros/humble/setup.bash
echo "$ROS_DOMAIN_ID"
~~~

If it does not match the robot's assigned domain, ask staff to correct **D**, not the robot. **Only after it matches**, run:

~~~bash
ros2 run tf2_ros tf2_echo odom base_footprint
~~~

Brief `Invalid frame ID` warnings may occur during discovery. If valid transforms do not follow, ask staff; **do not start teleop**. Note the starting translation and yaw. **Leave D running** during movement.

**Step 2 — Terminal C: start teleop.** Reuse C after closing RViz. Source ROS and check C's domain first:

~~~bash
source /opt/ros/humble/setup.bash
echo "$ROS_DOMAIN_ID"
~~~

If it does not match the robot, ask staff. **Only after it matches**, launch teleop:

~~~bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
~~~
Do not move yet. The teleop keys work only when **C has keyboard focus**.

**Step 3 — Terminal B: compare the request.** With the robot still, read a **fresh starting pose** in B, then watch velocity commands:

~~~bash
ros2 topic echo --once /odom --field pose.pose
ros2 topic echo /cmd_vel
~~~

In a `Twist` message, `linear.x` is requested forward speed (m/s) and `angular.z` is requested turning rate (rad/s). The echo may be blank until motion. Return focus to **C**, move slowly and briefly, then press Space/`s` to stop. **Space or `s` requests a stop**; releasing the movement key may not. Confirm the robot is stationary; if it fails to stop, **use the physical stop immediately and tell staff**. In B, look for nonzero then zero requests. Press Ctrl+C **in B only** to close the echo.

**Step 4 — Terminal B: compare the estimate.** With the robot stopped:

~~~bash
ros2 topic echo --once /odom --field pose.pose
~~~

Compare the two `/odom` poses (x, y and heading), the changing `odom → base_footprint` readings in D, and what you saw on the floor. The fixed `base_link → base_scan` mounting transform should not change. **Odometry is an estimate, not a physical distance measurement.**

**Step 5 — Shut down safely.** Once stationary, Ctrl+C in **C** to exit teleop, then Ctrl+C in **D** to stop TF. Check in B:

~~~bash
ros2 topic info /cmd_vel
~~~

Expect `Publisher count: 0` if no other controller is active. Do not launch another movement program while teleop is running. **Ctrl+C is a normal exit, not an emergency stop.**

**Checkpoint 4:** Explain how the **command**, **odometry estimate**, changing TF and **observed physical movement** compare. Why might they disagree? **Go further (optional):** why can SLAM correct `map → odom` while keeping local `odom` continuous?

---

## 5. Python investigation and the required four-sided square

Now test the question: can the robot return to its starting point, and how well can we measure the difference?

**Why are there two Python files?**

| File | Purpose |
|---|---|
| [`relative_move_math_walkthrough.py`](relative_move_math_walkthrough.py) | Safe maths model: ideal corners and overshoot, **no ROS connection or robot movement** |
| [`turtlebot3_relative_move_fixed.py`](turtlebot3_relative_move_fixed.py) | Real ROS controller: uses `/odom` feedback and publishes `/cmd_vel` |

The physical controller subscribes to `/odom` and publishes `/cmd_vel`. It does **not directly read** LiDAR, IMU or TF2, avoid obstacles, or provide the cancellable ROS action from Lab 2.

Follow **predict → inspect → run and measure → explain**. Inspect the ROS controller without editing it; you'll write nodes in Lab 4. **Only run the physical controller** after staff confirm its manual-input procedure and stopping checks.

### 5.1 Predict the square (no robot movement)

Keep the TurtleBot3 still. Imagine starting at `(0,0)`, facing east (`0°`). This is not the real robot's `/odom` pose.

Each leg: move **forward 1 m in the direction faced**, then **turn left 90°** (`x=1`, `y=0`, `theta=90°`). The turn changes the next direction, not position.

| Leg | Position after moving | Heading after turning |
|---|---|---|
| 1 (example) | `(1,0)` | North (`90°`) |
| 2 | ? | ? |
| 3 | ? | ? |
| 4 | ? | ? |

Use east **+X**, north **+Y**, west **−X**, south **−Y**. Sketch the square or work out the missing entries (2 minutes). Then check with the **safe Python model in 5.2**. `360°` and `0°` mean the same heading.

### 5.2 Check the folder and run the safe Python model

On the miniPC, open your supplied **Lab 3 folder in the course workspace** in **Terminal C** using the **file manager**. If you cloned this repository, use `cd 3` **from its root**; otherwise use the folder staff supplied.

~~~bash
pwd
ls README.md relative_move_math_walkthrough.py turtlebot3_relative_move_fixed.py
~~~

If a file is missing, ask staff. Run each command **separately**:

~~~bash
python3 -B relative_move_math_walkthrough.py
python3 -B relative_move_math_walkthrough.py --turn -90
python3 -B relative_move_math_walkthrough.py --side 0.5
~~~

Check corners against 5.1. Predict how right turns (`--turn -90`) or shorter sides (`--side 0.5`) change the path. The repeated overshoot figures are a **fixed example**, not measured errors.

**Independent transfer:** start facing **north**, move forward **0.5 m**, turn **right 90°**, twice. Predict displacement and final heading first.

### 5.3 Why can the robot overshoot?

**5.2 example:** target **1.000 m**, progress **1.013 m**, sideways error **0.027 m**.

| Check (0.01 m threshold) | Value | Result |
|---|---|---|
| Unsigned distance (hypothetical rule) | 0.030 m | Would keep moving |
| Signed remaining (controller rule) | −0.013 m | End forward leg |

Unsigned distance stays positive after overshoot; signed remaining becomes negative. Open [the controller](turtlebot3_relative_move_fixed.py) in a text editor; find `generate_path` (do not run it):

~~~python
progress = (
    dx * self.segment_unit_x +
    dy * self.segment_unit_y
)
remaining = self.segment_length - progress
~~~

`progress` tracks movement **along the planned direction**, not sideways. `go_straight` requests forward motion if `remaining > 0.01`; otherwise it advances the step and publishes zero velocity.

**Required 3-minute code investigation (read only; do not run yet):** Find `create_publisher`, `create_subscription`, `odom_callback` and `go_straight`. Identify the command and odometry topics, stored pose fields, and why sideways error may remain.

Signed progress cannot ensure odometry freshness, obstacle avoidance or safe stopping. Do **not** edit or execute the controller at this stage.

### 5.4 Prepare for the physical square

The **main robot experiment** repeats (`x=1 m, y=0 m, theta=90°`) four times. A manual four-leg run succeeded on one robot; **staff will confirm for each robot**. **Go/no-go check only; not permission to move.**

**Ask staff:** **Has the manual-input four-leg procedure been checked and approved for this robot?** Approval covers **this file, bad inputs, fresh odometry during prompts and physical stopping**. If **not yet**, **do not start the controller**. Mark `NOT RUN`; use **Section 5.6**.

**Only if approved, check together:**
- Clear 1 m × 1 m route with **robot clearance and stopping margin**; mark starting centre and heading.
- Operator, observer and reachable **physical stop** ready.
- Battery/bringup healthy; teleop exited; robot still; no other `/cmd_vel` publisher; correct domain and **live** `/odom`. After restart, take a new baseline.

**If a check fails, do not start.** Tell staff; use 5.6. Ctrl+C is **not** the physical emergency stop.

### 5.5 Execute four separate relative movements

**Run this physical exercise only after staff approve your robot in Section 5.4.** Mark the robot's starting centre and heading on the floor. Keep the route clear, with an observer and a reachable physical stop. Make sure teleop and every other movement controller have exited.

**Step 1 — Check the robot in Terminal B.** Do this after positioning the robot at its final starting mark:

~~~bash
source /opt/ros/humble/setup.bash
echo "ROS domain: ${ROS_DOMAIN_ID:-unset}"
ros2 topic info /cmd_vel
timeout 6s ros2 topic echo --once /sensor_state --field torque
timeout 6s ros2 topic echo --once /battery_state --field voltage
timeout 6s ros2 topic echo --once /odom --field pose.pose
~~~

Ask staff to verify the assigned ROS domain, `Publisher count: 0` on `/cmd_vel`, torque `True`, sufficient battery voltage, and a live `/odom` reading. If anything is missing, **do not launch the controller**. Record the starting odometry X, Y and orientation for your own trial; the values need not be zero.

**Step 2 — Start the controller in Terminal C.** First open the Lab 3 student folder identified in Section 5.2, then check:

~~~bash
pwd
ls turtlebot3_relative_move_fixed.py
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
echo "ROS: $ROS_DISTRO; domain: ${ROS_DOMAIN_ID:-unset}"
~~~

Confirm that the file is present, ROS is Humble and C uses your robot's assigned domain. With staff approval, launch:

~~~bash
python3 -B turtlebot3_relative_move_fixed.py
~~~

Wait until `Input x:` appears. **A prompt does not prove that odometry is fresh:** while waiting for keyboard input, this controller pauses ROS callbacks. Do not reposition the robot between prompts. Tell staff if you had a long pause or interrupted the sequence.

**Step 3 — Execute one leg, then check its result.** In Terminal C, enter these **three numbers separately**, pressing Enter after each:

~~~text
Input x: 1
Input y: 0
Input theta (deg): 90
~~~

This means move approximately 1 m forward relative to the robot's current heading, then turn 90° left. The controller now rejects non-finite X/Y inputs and values outside −1 to +1 m, but this is **not** a collision or stale-odometry safeguard. If you mistype an input, **do not complete the remaining prompts**; ask staff rather than trying to correct the path yourself.

Wait for the robot to finish its forward movement and turn. When it has **physically stopped** and the next `Input x:` prompt appears, measure that corner's estimated pose in **Terminal B**:

~~~bash
timeout 6s ros2 topic echo --once /odom --field pose.pose
echo "ODOM_EXIT=$?"
~~~

An `ODOM_EXIT=0` result means a message was received. Record X, Y and orientation; it does **not** prove physical accuracy or that the robot has stopped. If no message arrives, stop the trial and tell staff. The observer and staff should confirm the robot is stationary and the next side is clear.

**Repeat the same three entries for Legs 2, 3 and 4, one complete leg at a time.** After each leg, wait for the robot to stop and record that corner in B before starting the next. **Do not paste all 12 inputs at once. Do not press Ctrl+C after legs 1–3.**

**Step 4 — Finish after Leg 4 (normal completion).** Once the fourth leg has stopped, record its final odometry pose in Terminal B. When Terminal C displays another `Input x:` prompt, **do not enter a fifth leg**. Press **Ctrl+C once in C** to exit normally.

In **Terminal B**, verify shutdown:

~~~bash
ros2 node list
ros2 topic info /cmd_vel
~~~

The controller node should be absent, `/cmd_vel` should have `Publisher count: 0`, and the robot must remain stationary. Leave robot bringup running in A. Staff must confirm that it is safe to approach the robot for the physical measurement in Section 5.6.

**If the robot moves unexpectedly or fails to stop**, use the **independent physical stop immediately**, tell staff and mark the trial `ABORTED`. **Ctrl+C is not an emergency stop**, and no further commands should be entered.

### 5.6 Measure the result rather than guessing

**1. Record your trial.** **For a completed trial, keep just these values in your own notes** from Section 5.5. Record the start and each stopped corner from `/odom`; your group may start at different coordinates.

| Stage | `/odom` X (m) | `/odom` Y (m) | Orientation (z,w) / physical observation |
|---|---:|---:|---|
| Start | | | |
| After Leg 1 | | | |
| After Leg 2 | | | |
| After Leg 3 | | | |
| After Leg 4 | | | |

**2. Calculate odometry closure.** In **Terminal B**, run this offline Python command. When prompted, enter **your own** starting X and Y and final X and Y in metres:

~~~bash
python3 -B -c '
from math import hypot
sx, sy = float(input("Start X (m): ")), float(input("Start Y (m): "))
fx, fy = float(input("Final X (m): ")), float(input("Final Y (m): "))
e = hypot(fx-sx, fy-sy)
print(f"Odometry closure: {e:.4f} m ({e*100:.2f} cm)")
'
~~~

The calculation `E_odom = sqrt(dx² + dy²)` gives **estimated closure**: the distance between the starting and final *odometry estimates*. It is **not physical accuracy**. Use Section 4.1 to convert quaternions to yaw if you wish to compare headings. Never mix readings from different odometry resets or runs.

**3. Measure the physical result if permitted.** After staff confirm the robot is safe to approach, use a ruler or tape measure from the **marked starting centre** to the **final centre of the stationary robot**. Do not move it first. If you can only judge its position visually, write **“near start; not measured”** instead of inventing a distance.

**If the physical square is not approved or not run:** analyse this **historical instructor odometry-only** record from the **15 October 2026 supervised automatic-input test**. These readings are **not from your group**:

| Stage | Odometry X (m) | Odometry Y (m) |
|---|---:|---:|
| Start | −0.866113 | +0.763357 |
| After Leg 1 | +0.123217 | +0.982969 |
| After Leg 2 | −0.155072 | +1.960479 |
| After Leg 3 | −1.132116 | +1.680687 |
| After Leg 4 | −0.852211 | +0.705099 |

Calculate its odometry closure using the first and last rows. There was **no ruler-measured physical closure**: you **cannot calculate numerical physical closure** or physical-versus-odometry error. The record also lacks headings, so you cannot calculate yaw closure.

**Checkpoint 5:** Explain what your recorded positions show and what they cannot prove. If the exercise was not approved, mark `NOT RUN` and analyse the instructor data instead. If movement required a safety stop, mark `ABORTED` and tell staff. State any measurements that are missing.

---

## 6. Before leaving — can you explain what happened?

**Exit check:** Think through these **new situations independently** before discussing them with your group. Separate **observations from hypotheses**:

1. **IMU mounting:** an IMU is rotated 90° on its bracket while the body is still. Which transform changes, and why would uncorrected readings mislead?
2. **RViz:** Fixed Frame is `map`, but no node provides `map → odom`. What would you check, and what safe display change could help?
3. **Movement:** `/cmd_vel` is nonzero, but `/odom` barely changes. Give two explanations and a test to distinguish them.
4. **Geometry:** starting facing **west**, move forward **0.25 m** and turn left 90°, **twice**. Predict final displacement and heading. What does signed progress still not guarantee?
5. **Evidence:** using your measurements if available, or instructor odometry-only data otherwise, what can you conclude about physical versus estimated position? What extra measurement would you need?

Check that the robot is stationary, close all movement controllers and follow staff shutdown instructions.

**Next:** Lab 4 writes ROS nodes; Lab 5 studies mapping. Both build on this distinction: **command ≠ estimate ≠ physical measurement**.

### If something does not work

| Problem | First check |
|---|---|
| `No such file or directory` for Python | Use `pwd` and `ls` in the folder checked in 5.2 |
| `ros2: command not found` | Source ROS Humble as in Labs 1–2; ask staff if the provisioned environment differs |
| No robot nodes or wrong robot | Compare assigned `ROS_DOMAIN_ID` on both machines; don't duplicate bringup |
| `No transform` in RViz or `tf2_echo` waiting | Actual frame names, `/tf`, RViz Fixed Frame; don't invent transforms |
| No LiDAR/IMU samples or laser points | Publisher, offered QoS, compatible **Best Effort** and RViz Status; silence isn't proof of failure |
| Unexpected motion or command publisher | Physical stop, **ABORTED**, no new commands; contact staff |

This Lab 3 folder contains this README and two Python files.
