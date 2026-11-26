# Lab 7

## File tree

```text
lab_7/
├── README.md                    # Lab worksheet
├── .env.example                 # Per-team lab settings
├── Makefile                     # Lab commands
├── exercises/
│   ├── aruco_reader.py          # Task 1 marker detector
│   ├── autonomous_search.py     # Task 2 mission
│   ├── planner.py               # A* planner reused from Lab 6
│   └── explore_starter.launch.py # Optional Task 3 exploration
├── config/
│   ├── sim_mission.yaml         # SIM target and poses
│   ├── real_mission.yaml        # REAL target, marker key and poses
│   └── static/                  # Supplied lab settings
├── launch/                      # Lab launch files
├── packages/                    # Lab camera, planner and mission code
├── maps/                        # Lab map and arena diagrams
└── tests/                       # Exercise checks
```

## Make commands

Run from your `labs/7` directory on the student computer. Copy `.env.example` to `.env` if needed and enter your allocated `ROBOT_NAME` and `ROS_DOMAIN_ID`. Use `USE_SIM_TIME=true` and `ROS_LOCALHOST_ONLY=1` for SIM. For REAL, set `USE_SIM_TIME=false` and use the demonstrator’s localisation setup.

Run `make help` for the command list. Keep long-running commands in separate terminals.

| Task | Commands |
| --- | --- |
| **1. Read signs and record poses** | In SIM, start `make sim` and `make localise`. Run `make decode`, `make rviz` and `make check`. Drive with `make teleop` and use `make pose` at the start, each sign view and each parking spot. |
| **2. Run the mission** | Run `make build`. Return to A on the same map and stop teleop. Keep localisation and the camera running. Start `make nav`, then run `make mission`. |
| **3. Explore (optional)** | In SIM, stop the Task 2 processes and restart `make sim`. Start `make mapping`, `make rviz` and `make decode`. Once the map updates, start `make nav`, then `make explore`. For REAL, follow the supervised Task 3 procedure in `README.md`. |


### Command reference

| Command | Purpose |
| --- | --- |
| `make localise` | Find the robot’s position on the supplied map. |
| `make mapping` | Build a live map for exploration. |
| `make decode` | Process the camera stream and publish live marker readings. |
| `make check` | Check that fresh camera frames are arriving. |
| `make pose` | Show the map coordinates and heading to record in the mission file. |
| `make nav` | Start Nav2 so it can accept navigation goals. |
| `make mission` | Visit the recorded views and use fresh sign readings to choose a parking goal. |
