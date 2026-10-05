# Exercise 1 — ROS 2 Fundamentals and System Awareness

This is the hands-on exercise for Lab 1. It assumes your environment already passes the checks in [Section 4.1 of the main Lab 1 README](../README.md#41-check-your-environment-works) — if `talker`/`listener` don't work yet, sort that out first.

The physical robot (SSH, bringup, teleop) is covered separately in [Section 5 of the main README](../README.md#5-your-first-robot-ssh-bringup-teleop) — this exercise is entirely software, run on the lab miniPC or your own machine.

The aim is confidence and good habits — build, source, inspect — not speed.

---

## What you'll be able to do

By the end of this exercise, you should be able to:

- Explain what a ROS 2 node and topic are, in your own words
- Run existing ROS 2 nodes and inspect the ROS graph
- Write, build and run a minimal publisher node
- Set and read a parameter on a running node
- Use the basic `ros2` CLI tools to debug what's going on

---

## Before you start

You need a sourced ROS 2 Humble environment (any of the routes in [Section 4](../README.md#4-setting-up-your-environment)) and a ROS 2 workspace to build packages in.

Your group keeps its work in its own workspace folder on the miniPC, named after your group number: group 1 uses `~/group1_ws`, group 2 uses `~/group2_ws`, and so on. Record it once in `~/.bashrc`, the same way you set `ROS_DOMAIN_ID` in [Section 4.2](../README.md#42-set-your-ros_domain_id), so every terminal knows where it is:

```bash
echo 'export GROUP_WS=~/group<your group number>_ws' >> ~/.bashrc
source ~/.bashrc
echo $GROUP_WS
```

Group 1 writes `export GROUP_WS=~/group1_ws`. The angle brackets just mark the gap you fill in. If someone in your group has already done this on your miniPC, `echo $GROUP_WS` prints the path and you can skip it.

Then create the workspace, if it does not exist yet:

```bash
mkdir -p $GROUP_WS/src
cd $GROUP_WS
colcon build
source install/setup.bash
```

Every command below uses `$GROUP_WS`, so it works unchanged for every group.

If something here doesn't work and it looks environment-related rather than exercise-related, check the troubleshooting table in [Section 8 of the main Lab 1 README](../README.md#8-when-something-goes-wrong) before asking for help.

---

## Part 1 — Run existing ROS 2 nodes

### Task 1.1: Talker and listener

In one terminal:

```bash
ros2 run demo_nodes_cpp talker
```

In a second terminal:

```bash
ros2 run demo_nodes_cpp listener
```

You should see messages published by the talker and received by the listener. Stop both with Ctrl+C when you're done.

### Task 1.2: Inspect the ROS graph

Run the talker and listener again, then in a third terminal:

```bash
ros2 node list
ros2 topic list
ros2 topic info /chatter
```

Answer these in your own words:

- Which nodes exist?
- Which topic connects them?
- Who publishes and who subscribes?

**Optional (recommended):** sketch a small diagram showing the nodes and the `/chatter` topic between them.

---

## Part 2 — Write a minimal publisher (Python)

You'll create a small Python package with a single publisher node.

### Task 2.1: Create a package

```bash
cd $GROUP_WS/src
ros2 pkg create --build-type ament_python lab01_basics
```

Build and source again:

```bash
cd $GROUP_WS
colcon build
source install/setup.bash
```

### Task 2.2: Create the publisher node

Create this file:

`$GROUP_WS/src/lab01_basics/lab01_basics/simple_publisher.py`

Use this exact starter code:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    def __init__(self):
        super().__init__("simple_publisher")

        # Parameter controls the message content
        self.declare_parameter("message_text", "Hello from Lab 01")

        self.pub = self.create_publisher(String, "/lab01/chatter", 10)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        msg_text = self.get_parameter("message_text").get_parameter_value().string_value
        msg = String()
        msg.data = msg_text
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")


def main():
    rclpy.init()
    node = SimplePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### Task 2.3: Register the executable

Edit `$GROUP_WS/src/lab01_basics/setup.py`. Find the `entry_points` section and set it to:

```python
entry_points={
    'console_scripts': [
        'simple_publisher = lab01_basics.simple_publisher:main',
    ],
},
```

Build and source again:

```bash
cd $GROUP_WS
colcon build
source install/setup.bash
```

### Task 2.4: Run your node and verify

Terminal A (run the publisher):

```bash
ros2 run lab01_basics simple_publisher
```

Terminal B (observe the topic):

```bash
ros2 topic echo /lab01/chatter
```

You should see your messages arriving. Stop both with Ctrl+C.

---

## Part 3 — Parameters

Run your node with a different message:

```bash
ros2 run lab01_basics simple_publisher --ros-args -p message_text:="Hello from parameters"
```

Verify in another terminal:

```bash
ros2 param list /simple_publisher
ros2 param get /simple_publisher message_text
```

---

## Part 4 — Launch files (extension)

Write a launch file that starts `simple_publisher` and sets `message_text`. Launch files get proper treatment in Lab 2 — this is a chance to see one early, not a requirement. The [ROS 2 tutorial on creating launch files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html) covers everything you need.

---

## Common issues

- Forgot to `source install/setup.bash` after building
- Edited the code but didn't rebuild with `colcon build`
- Echoing the wrong topic name — this exercise uses `/lab01/chatter`, not `/chatter`

For anything else, see the troubleshooting table in [Section 8 of the main Lab 1 README](../README.md#8-when-something-goes-wrong).

