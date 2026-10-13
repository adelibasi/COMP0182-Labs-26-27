# Lab 2 — Services, Actions, and the ROS 2 Communication Model

Lab 1 covered **nodes and topics**. This lab introduces two more ROS 2 communication patterns: a one-shot request and response (a **service**), and a goal that can take time, report progress, and be cancelled (an **action**). Everything runs in **turtlesim** so you can see the communication patterns working without a physical robot.

Next week you will be back on the real robots.

You need a working **ROS 2 Humble** environment from [Lab 1, Section 4](../1/README.md#4-setting-up-your-environment), including the `turtlesim` package. **Section 5.2** also needs `xterm` on Ubuntu because the supplied launch file uses it. You do not need `xterm` for the earlier exercises.

> **Before you start:** Follow the instructions in order. Commands assume you have completed Lab 1. If you are on a UCL lab miniPC or using RoboStack/pixi rather than a system-wide Ubuntu installation, follow the setup route for **your machine**. Ask a demonstrator if the environment differs; do not mix ROS installations in one terminal.

---

## The goal for this lab session

- [ ] Call a service from the CLI and see its response.
- [ ] Send an action goal, watch feedback stream in, and cancel an active goal.
- [ ] Complete the integration task: turtle1 draws in red, turtle2 moves to `(8, 8)`, and turtle1 faces north.
- [ ] Attempt at least one extension.

By the end of the lab, you should be able to:

1. Use CLI tools to inspect a live ROS 2 computation graph.
2. Explain when to use a **topic**, **service**, or **action**, and why.
3. Call services and send action goals from the command line.
4. Recognise and fix common mistakes with these tools.

**How to use the terminals:** **Terminal A** runs turtlesim and stays open. **Terminal B** is where you enter the other commands. Commands in this worksheet are executed **one at a time**. In particular, do not paste a new command into a terminal while a long-running command occupies it.

---

## 1. Quick topics recap

### 1.1 Make sure your environment is active

Open **Terminal A**.

On your group's lab miniPC, ROS 2 Humble is already set up, as in [Lab 1, Section 4.1](../1/README.md#41-check-your-environment-works). If `printenv ROS_DISTRO` does not print `humble`, run:

```bash
source /opt/ros/humble/setup.bash
```

If you are working on your own machine with RoboStack/pixi ([Lab 1, Section 4.4](../1/README.md#44-optional-ros-2-on-your-own-machine)), activate that environment instead:

```bash
pixi shell -e humble
```

### 1.2 Start turtlesim

In **Terminal A**, run:

```bash
ros2 run turtlesim turtlesim_node
```

**Expected:** A turtlesim window opens with one turtle near the centre.

Leave Terminal A running.

Now open **Terminal B** and activate your ROS 2 environment using the same command from Section 1.1.

You're ready to continue to **Section 1.3 — Inspect the graph**.

### 1.3 Inspect the graph

In **Terminal B**, list the running nodes:

```bash
ros2 node list
```

You should see:

```text
/turtlesim
```

List topics:

```bash
ros2 topic list
```

A fresh turtlesim normally exposes:

```text
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

Inspect the turtle's pose topic:

```bash
ros2 topic info /turtle1/pose
```

Typical output before other pose subscribers are running:

```text
Type: turtlesim/msg/Pose
Publisher count: 1
Subscription count: 0
```

Read one pose message:

```bash
ros2 topic echo /turtle1/pose --once
```

**Expected:** One message showing `x`, `y`, `theta`, `linear_velocity`, and `angular_velocity`, then the command finishes. `theta` is an angle in **radians**.

Publish one forward velocity command:

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

**Expected:** Turtle1 moves briefly and leaves a white trail. A single message is not a continuous driving command: turtlesim stops moving when the velocity command times out.

### 1.4 Three communication patterns

```text
┌─────────────┐    /turtle1/pose (topic stream)    ┌────────────┐
│             │ ─────────────────────────────────► │ subscriber │
│  turtlesim  │                                    └────────────┘
│    node     │    /spawn (request → response)     ┌────────────┐
│             │ ◄─────────────────────────────────► │   client   │
│             │                                    └────────────┘
│             │    /rotate_absolute (goal,          ┌────────────┐
│             │     feedback, result)              │   client   │
│             │ ◄─────────────────────────────────► │            │
└─────────────┘                                    └────────────┘
```

| **Pattern** | **Direction** | **Blocking?** | **Use when…** |
|---|---|---|---|
| **Topic** | One-way publisher → subscriber | No reply per message | Sensor data, velocity commands |
| **Service** | Request → response | CLI waits for reply | One-shot operations: spawn, clear, teleport |
| **Action** | Goal → feedback → result | Can run asynchronously | Tasks that take time and may be cancelled |

An action goal uses the *current state* of the turtle. Keep this in mind when you reach Section 3.

---

## 2. Services

A **service** lets a client ask a server to perform an operation and return a single response. In a ROS `.srv` definition, the `---` separator divides the **request** fields from the **response** fields.

```text
Client                       Server
  │ ─────── request ────────► │
  │                            │  (processes)
  │ ◄─────── response ──────── │
```

### 2.1 List available services

With turtlesim still running, enter in **Terminal B**:

```bash
ros2 service list
```

At minimum, you should see services such as:

```text
/clear
/kill
/reset
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
```

Additional ROS parameter services are normal.

### 2.2 Get the type of a service

```bash
ros2 service type /spawn
```

Expected:

```text
turtlesim/srv/Spawn
```

### 2.3 Inspect the service interface

```bash
ros2 interface show turtlesim/srv/Spawn
```

The key fields are:

```text
float32 x
float32 y
float32 theta
string name
---
string name
```

Fields **above** `---` are what you send; the field **below** is what comes back. Actual interface output can contain explanatory comments.

### 2.4 Clear the canvas

```bash
ros2 service call /clear std_srvs/srv/Empty "{}"
```

**Expected:** The white trail from Section 1.3 disappears. `/clear` removes drawings; it does not reset the turtle's position.

### 2.5 Spawn a second turtle

First check whether turtle2 is already present:

```bash
ros2 topic list
```

If you already see `/turtle2/pose`, **skip the next spawn command**. Otherwise run:

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

**Expected:** The service returns `name='turtle2'`, and a second turtle appears near `(2, 2)`. Spawning another turtle with the same name will fail; do not treat that as a simulator fault.

### 2.6 Change the pen colour

Set turtle1's pen to red, width 5:

```bash
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 5, 'off': 0}"
```

Move it briefly:

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}" --once
```

**Expected:** Turtle1 draws a **red** trail or arc. The `off: 0` field means the pen is enabled. The apostrophes around `'off'` in the YAML are intentional.

### 2.7 Teleport a turtle

```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5.5, y: 5.5, theta: 0.0}"
```

Verify:

```bash
ros2 topic echo /turtle1/pose --once
```

**Expected:** Turtle1 is near `(5.5, 5.5)` and has heading `theta ≈ 0.0` radians. This sets up a known starting direction for the next section.

> **Why is there an unexpected straight line?** In turtlesim, teleporting a turtle with its pen enabled may draw a line from its previous position. This does not mean the teleport failed. Use `/clear` if you want a clean canvas.

Before continuing, check that you saw the `/spawn` response and turtle1's red trail.

---

## 3. Actions

An **action** is useful when an operation takes time. A client sends a goal, receives progress **feedback**, and eventually receives a **result**. An active goal can also be cancelled.

```text
Client                          Server
  │ ───────── goal ───────────► │
  │ ◄──────── feedback ──────── │  (repeats while moving)
  │ ───────── cancel ─────────► │  (optional while active)
  │ ◄──────── result ────────── │  (finished or cancelled)
```

An action interface has three sections separated by `---`: **goal**, **result**, **feedback**.

### 3.1 List available actions

```bash
ros2 action list
```

You should see:

```text
/turtle1/rotate_absolute
/turtle2/rotate_absolute
```

Turtle2's action exists because you spawned it in Section 2.5.

### 3.2 Inspect action info

```bash
ros2 action info /turtle1/rotate_absolute
```

**Expected:** One action server, `/turtlesim`, and usually zero action clients when no goal command is running.

To check its type:

```bash
ros2 action list -t
```

The type of `/turtle1/rotate_absolute` is `turtlesim/action/RotateAbsolute`.

### 3.3 Inspect the action interface

```bash
ros2 interface show turtlesim/action/RotateAbsolute
```

The fields are:

```text
# GOAL
float32 theta
---
# RESULT
float32 delta
---
# FEEDBACK
float32 remaining
```

- **Goal `theta`** is the turtle's **absolute target heading** in radians, not how many turns to complete.
- **Feedback `remaining`** tells you how much rotation is left while the goal is running.
- **Result `delta`** is a **signed rotation displacement**, not the turtle's final heading. In this Humble turtlesim implementation, a positive change in the pose heading can produce a negative `delta`. Check `/turtle1/pose` to confirm the final direction; do not assume `delta` equals the goal angle.

### 3.4 Send a goal with feedback

Make sure turtle1 starts facing approximately zero. You can restore the heading with:

```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5.5, y: 5.5, theta: 0.0}"
```

Send the goal:

```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}" --feedback
```

**Expected:** The goal is accepted, `Feedback:` appears as `remaining` decreases, then the result reports `SUCCEEDED`. The exact feedback count and `delta` depend on timing and starting orientation.

Check the final heading:

```bash
ros2 topic echo /turtle1/pose --once
```

The heading should be **near `1.57` radians** (north).

### 3.5 Send a long goal and cancel it

For a reliable cancellation example, first put turtle1 at a **known starting heading**:

```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5.5, y: 5.5, theta: 0.0}"
```

Now send an approximately half-turn goal:

```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 3.14}" --feedback
```

**While feedback is still streaming**, press **Ctrl+C once in Terminal B**, where the action command is running. **Do not** press Ctrl+C in Terminal A, which is running turtlesim.

**Expected:** A cancellation acknowledgement and final status `CANCELED`. Confirm the heading:

```bash
ros2 topic echo /turtle1/pose --once
```

The turtle should have stopped **between** the initial and requested headings if you cancelled in time. If the terminal already said `SUCCEEDED`, it finished before cancellation: reset its heading and try again. Do not mark a completed action as cancelled.

> **Why not `theta: 6.28`?** `theta` specifies an *absolute* heading. `6.28` is approximately a full revolution, equivalent to nearly zero; starting from zero, this may finish almost immediately rather than making a long turn.

### 3.6 Send a goal without feedback

Place turtle1 at a heading different from the next target:

```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5.5, y: 5.5, theta: 1.5708}"
```

Send an action goal **without** `--feedback`:

```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 0.0}"
```

**Expected:** The CLI prints the accepted goal and final result (`SUCCEEDED`), but **no `Feedback:` lines**. The turtle still rotates; the command simply does not display the feedback stream.

```bash
ros2 topic echo /turtle1/pose --once
```

The new heading should be close to zero. A small residual angle is normal.

Before moving on, make sure you have observed **feedback**, a genuine **cancellation**, and a goal completed **without** feedback printed.

---

## 4. Integration task

**Scenario:** You are setting up a two-turtle demonstration. Configure turtle1 with a red pen, move turtle2 to `(8, 8)`, and orient turtle1 to face north (approximately `1.57` radians). Use **ROS CLI calls only**.

You can use your existing simulator; **do not start another turtlesim instance**.

**Step 1 — Red pen for turtle1:**

```bash
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 3, 'off': 0}"
```

Drive briefly to check the colour:

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" --once
```

**Expected:** A short **red trail** appears behind turtle1.

**Step 2 — Move turtle2 to `(8, 8)`:**

If turtle2 exists from Section 2.5, **do not spawn it again**. If you did not spawn turtle2 earlier, use:

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

Then teleport turtle2:

```bash
ros2 service call /turtle2/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 8.0, y: 8.0, theta: 0.0}"
```

Check its pose:

```bash
ros2 topic echo /turtle2/pose --once
```

**Expected:** `x ≈ 8.0` and `y ≈ 8.0`. Turtle2 may leave a **white diagonal line** from its old location because its pen was enabled; this is expected.

**Step 3 — Rotate turtle1 to face north (`θ = π/2 ≈ 1.57`):**

```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.5708}" --feedback
```

Check its pose:

```bash
ros2 topic echo /turtle1/pose --once
```

**Done when:** You have seen turtle1's red trail, turtle2's pose is near `(8, 8)`, and turtle1's final `theta` is near `1.57` radians. The red trail and the possible white teleport line are **different** observations.

---

## 5. Extensions

These exercises build on Sections 2–4. Try at least one; explain **what happened and why**, not only which commands you entered.

### 5.1 Cancellation and state

Find out what happens to a turtle's state when an action is cancelled during execution:

1. Check turtle1's heading with `ros2 topic echo /turtle1/pose --once`.
2. Set a known starting heading:

   ```bash
   ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5.5, y: 5.5, theta: 1.70}"
   ```

3. Send a goal to absolute heading `4.71` radians:

   ```bash
   ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 4.71}" --feedback
   ```

4. Press **Ctrl+C in Terminal B while feedback is still streaming**. Check for `CANCELED`; then read the new pose:

   ```bash
   ros2 topic echo /turtle1/pose --once
   ```

5. Without teleporting or resetting, send the **same goal again**, and allow it to finish:

   ```bash
   ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 4.71}" --feedback
   ```

**Think about:** Is the heading after cancellation the original heading, the goal heading, or somewhere between? Does the second goal start from the **retained** position? Compare the **magnitude** of its result `delta` with the previous action's last `remaining` value. Account for controller tolerance and **angle wrapping**: `4.71` radians may appear as approximately `−1.57` in `/turtle1/pose`.

### 5.2 Launch files

A launch file can start several nodes with one command. The supplied [`launch/turtlesim_launch.py`](launch/turtlesim_launch.py) starts `turtlesim_node` and `turtle_teleop_key`.

**Step 1 — Check the extra requirement (Ubuntu/Linux):**

In **Terminal B**, run:

```bash
command -v xterm
```

**Expected:** A path such as `/usr/bin/xterm`. The supplied launch file uses `prefix='xterm -e'` to give keyboard teleoperation its own terminal window. This is a requirement of **this launch file**, not ROS 2 in general.

If `xterm` is missing and you **have permission to install software** on your Ubuntu machine, you can install it:

```bash
sudo apt update
sudo apt install xterm
```

On a managed lab computer without installation permission, **ask a demonstrator**. For non-Linux environments, get platform-specific guidance rather than assuming that `xterm` will work.

**Step 2 — Stop the current simulator:**

In **Terminal A**, press **Ctrl+C** once to stop the `ros2 run turtlesim turtlesim_node` command from Section 1.2. Wait for the TurtleSim window to close. In **Terminal B**, check:

```bash
ros2 node list
```

Your previously started `/turtlesim` node should be gone. If it is still there, **do not launch another copy**; check the existing terminal or ask a demonstrator.

**Step 3 — Launch from the Lab 2 directory:**

In **Terminal A**, go to the `2` directory in **your own course repository**, the directory containing this README. Check:

```bash
pwd
test -f launch/turtlesim_launch.py && echo "Launch file found"
```

If the file is not found, you are in the wrong directory. Navigate to the course repository's `2` folder before continuing. Start the launch:

```bash
ros2 launch launch/turtlesim_launch.py
```

**Expected:** Two windows appear: **TurtleSim** and an **`xterm` terminal** displaying keyboard controls. Click inside `xterm` so it has keyboard focus; try the arrow keys to move the turtle.

**Step 4 — Check the ROS graph immediately:**

In **Terminal B**, run:

```bash
ros2 node list
```

The two nodes started by the launch should be:

```text
/turtlesim
/teleop
```

Now check their connection:

```bash
ros2 topic info /turtle1/cmd_vel
```

**Expected for your isolated test:** message type `geometry_msgs/msg/Twist`, **one publisher** (teleop) and **one subscriber** (turtlesim). Additional unrelated nodes may appear on a shared ROS domain. A visible application window **does not by itself prove** ROS topics are discoverable.

**Step 5 — Shut down together:**

Press **Ctrl+C once in Terminal A**, where `ros2 launch` is running. Both the TurtleSim and `xterm` windows should close. In Terminal B:

```bash
ros2 node list
```

Neither `/turtlesim` nor `/teleop` from your launch should remain. During interruption, some subprocesses can report nonzero exit codes; the important checks are whether the applications have stopped and the nodes have disappeared. If windows stay open, ask a demonstrator rather than force-killing unrelated processes.

**Think about:** Open `launch/turtlesim_launch.py` in an editor. Which two executables does it start? If you ran them independently from separate terminals, would Ctrl+C in one stop the other? What `Node(...)` declaration would start a third installed ROS executable?

---

## 6. When something goes wrong

| **Symptom** | **Likely cause** | **What to do** |
|---|---|---|
| `ros2: command not found` | ROS 2 is not active in this terminal | Source your Lab 1 environment or activate the pixi environment if that is your route. |
| `Could not find package turtlesim` | Package missing or wrong ROS environment | Check `ros2 pkg executables turtlesim`, then ask a demonstrator. |
| No turtlesim node in Terminal B | Different ROS domains or simulator not running | Compare `echo "${ROS_DOMAIN_ID:-0}"` in both terminals and inspect Terminal A. |
| Service/action waits for a server | No accessible server in this domain | Check `ros2 node list` and `ros2 service list` or `ros2 action list`. |
| `unknown service type` or `unknown field name` | Mistyped type or request field | Check `ros2 service type <service>` and `ros2 interface show <type>`. |
| `turtle2` already exists | It was spawned earlier | Skip spawning it again. Do not kill another student's turtle. |
| Cancellation says `SUCCEEDED`, not `CANCELED` | The action finished before Ctrl+C | Set the known initial heading and retry while feedback is still appearing. |
| Unexpected long white line | A pen-enabled turtle teleported | This is normal; use `/clear` if you want to remove drawings. |
| `xterm` not found during launch | Extra executable required by supplied launch file | Check `command -v xterm`; install only if permitted, otherwise ask a demonstrator. |
| Launch file cannot be found | You are not in the Lab 2 folder | Run `pwd` and `test -f launch/turtlesim_launch.py`; change directory. |
| Teleop keys do nothing | Wrong window has keyboard focus | Click inside the `xterm` window and try again. |
| Windows are open but `ros2 node list` is empty | ROS discovery/communication unavailable or environment mismatch | Check domains in both terminals, inspect the launch terminal, and ask a demonstrator. Do not repeatedly launch more simulators. |
| A window stays open after Ctrl+C | A launched process did not stop correctly | Note what remains and ask a demonstrator; avoid force-killing unrelated processes. |
| `rqt_graph` looks empty | Display may be stale, or graph discovery is unavailable | Refresh it and compare with `ros2 node list`. |

If an exercise behaves differently, **check the actual output before continuing**. It is better to show a demonstrator the unexpected result than to guess what happened.

## Appendix A — Command reference

| **Concept** | **Key CLI command** | **What it shows** |
|---|---|---|
| List nodes | `ros2 node list` | Discovered ROS nodes |
| List topics | `ros2 topic list` | Active topic names |
| Topic information | `ros2 topic info <topic>` | Type and publisher/subscriber counts |
| Read a topic once | `ros2 topic echo <topic> --once` | One message snapshot |
| Publish once | `ros2 topic pub <topic> <type> <yaml> --once` | One message published |
| List services | `ros2 service list` | Service endpoints |
| Service type | `ros2 service type <service>` | Service interface name |
| Service interface | `ros2 interface show <type>` | Request and response fields |
| Call a service | `ros2 service call <service> <type> <yaml>` | Request and response |
| List actions | `ros2 action list` | Action endpoint names |
| Action info | `ros2 action info <action>` | Servers and clients |
| Send a goal | `ros2 action send_goal <action> <type> <yaml> --feedback` | Goal and live feedback |
| Launch several nodes | `ros2 launch <launch_file>` | Start a group of nodes |
| List parameters | `ros2 param list <node>` | Configurable parameters |
| Get/set a parameter | `ros2 param get` / `ros2 param set` | Read or update configuration |

## Appendix B — Before you finish

- [ ] Both terminals use the intended ROS 2 environment and matching domain.
- [ ] You observed a topic publisher and used a velocity command.
- [ ] You called `/clear`, `/spawn`, `/set_pen`, and `/teleport_absolute`.
- [ ] You inspected an action, watched feedback, and confirmed **`CANCELED`** for a live goal.
- [ ] You compared action goals with and without `--feedback`.
- [ ] You observed a red turtle1 trail and verified both final turtle poses.
- [ ] If attempting 5.2: you checked `xterm`, saw `/turtlesim` and `/teleop`, and stopped both with Ctrl+C.
