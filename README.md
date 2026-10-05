# COMP0182 – Real-World Multi-Agent Systems

ROS 2 Humble and TurtleBot3, taught as a progressive path from software fundamentals to real physical robots — building confidence with ROS 2 in simulation, then carrying the same workflows onto physical robots.

The labs live in numbered folders (`1/`, `2/`, …), one per session, each with its own README.

Labs are released here week by week; only the labs covered so far are published.

---

## Setup

See [Lab 1, Section 4](1/README.md#4-setting-up-your-environment) for environment setup.

---

## Teaching team (2026-27)

- **Module Leader:** Akin Delibasi
- **Module Deputy Leader:** Lei Gao
- **Teaching Assistants:** Adhish Rao, George McCormick-Rust

---

## Acknowledgements

These lab sheets were developed with a substantial contribution from **Aydin Orhan**, former Teaching Assistant on this module, who wrote and tested much of the material.

---

## Schedule (2026-27)

Lectures are on Wednesday mornings and labs on Thursday mornings.

| Lab | Date | Topic |
|---|---|---|
| 1 | Thu 8 Oct | [Getting set up, and meeting the problem](1/README.md) |
| 2 | Thu 15 Oct | Services, actions and the ROS 2 communication model |
| 3 | Thu 22 Oct | TF2, bringup, sensors and odometry |
| 4 | Thu 29 Oct | Python nodes and closed-loop control |
| 5 | Thu 5 Nov | SLAM with Cartographer |
| | Thu 12 Nov | Reading week |
| 6 | Thu 19 Nov | Waypoints and markers |
| 7 | Thu 26 Nov | Two robots, one network |
| 8 | Thu 3 Dec | Planning together with CBS |
| 9 | Thu 10 Dec | Open arena session: run your solutions, record videos and collect results |
| 10 | Thu 17 Dec | Challenge show. The report is due at 16:00 the same day |

This schedule may still change. Each lab's own README says what it actually covers.

---

## The challenge

The module ends with two tasks on the physical TurtleBot3s, in the arena shown in [Lab 1, Section 6](1/README.md#6-the-demo-two-robots-one-bottleneck).

- **Task 1: Target search (one robot).** The robot starts from a given point and visits the entrance of each of three rooms in turn. Each room holds an ArUco marker standing for a fruit. Your group is told which fruit to find, and the robot must read the markers and enter the right room.
- **Task 2: Multi-robot path finding (two or more robots).** The robots solve the swap you saw in the Lab 1 demo, without collision, using Conflict-Based Search (CBS) or another multi-agent path finding algorithm of your choice.

Labs 6 to 8 build the pieces for these tasks. Lab 9 is for running your solutions in the arena and collecting results, and Lab 10 is where you show them.
