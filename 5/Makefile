SHELL := /bin/bash
.DEFAULT_GOAL := help

USE_SIM_TIME ?= true
USE_MOCAP ?= false
GUI ?= true
TOPIC ?= /localization_pose
SPATIAL_PROXIMITY ?= true
FIXED_FRAME ?= odom
ESTIMATOR ?= rtabmap
DURATION ?= 120
OUT_DIR ?= output/runs

export ROBOT_NAME MAP USE_SIM_TIME USE_MOCAP GUI TOPIC SPATIAL_PROXIMITY
export FIXED_FRAME ESTIMATOR DURATION OUT_DIR RUN
export TURTLEBOT3_MODEL := burger

.PHONY: help sim mapping rviz teleop frontiers save-map localise covariance \
	record compare check-robot check-map check-run

help: ## show commands and required settings
	@printf '%s\n' \
		'Usage: make <command> ROBOT_NAME=<name> [MAP=maps/arena.yaml]' \
		'Optional: USE_SIM_TIME=false GUI=false TOPIC=/localization_pose' \
		'          SPATIAL_PROXIMITY=false FIXED_FRAME=map USE_MOCAP=true' \
		'Record:   ESTIMATOR=rtabmap|amcl DURATION=120 OUT_DIR=output/runs' \
		'Compare:  make compare RUN=output/runs/<your-run>' \
		'Inspect:  make -n <command> to see the underlying command' ''
	@awk 'BEGIN { FS = ":.*## " } \
		/^[a-zA-Z0-9_-]+:.*## / { printf "  %-12s %s\n", $$1, $$2 }' $(MAKEFILE_LIST)

sim: check-robot ## start the named Burger in the Classic demo world
	ros2 launch ./launch/sim.launch.py "robot_name:=$$ROBOT_NAME" "gui:=$$GUI"

mapping: check-robot ## start RTAB-Map with a fresh database
	ros2 launch ./launch/rtab.launch.py "robot_name:=$$ROBOT_NAME" "use_sim_time:=$$USE_SIM_TIME" "use_mocap:=$$USE_MOCAP" "spatial_proximity:=$$SPATIAL_PROXIMITY"

rviz: check-robot ## open the named robot's RViz view
	ros2 launch ./launch/rviz.launch.py "robot_name:=$$ROBOT_NAME" "use_sim_time:=$$USE_SIM_TIME" "fixed_frame:=$$FIXED_FRAME"

teleop: check-robot ## drive the named robot with the keyboard
	ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r "cmd_vel:=/tb_$$ROBOT_NAME/cmd_vel"

frontiers: ## highlight unexplored boundaries; never drives the robot
	python3 exercises/frontier_explorer.py --ros-args -p "use_sim_time:=$$USE_SIM_TIME"

save-map: check-map ## export /map as PGM/YAML without replacing existing files
	bash ./scripts/save-map.sh "$$MAP"

localise: check-robot check-map ## localise on MAP; set 2D Pose Estimate in RViz
	ros2 launch ./launch/amcl.launch.py "robot_name:=$$ROBOT_NAME" "map_file:=$$MAP" "use_sim_time:=$$USE_SIM_TIME"

covariance: ## optionally plot pose covariance from TOPIC
	python3 scripts/rtab_covariance.py "$$TOPIC"

record: check-robot ## record simulated truth, odometry and estimated poses
	python3 scripts/record_poses.py --robot-name "$$ROBOT_NAME" --estimator "$$ESTIMATOR" --duration "$$DURATION" --out-dir "$$OUT_DIR"

compare: check-run ## check a recording and report trajectory errors
	python3 scripts/compare_poses.py "$$RUN"

check-robot:
	@if [[ ! "$$ROBOT_NAME" =~ ^[A-Za-z0-9_]+$$ ]]; then \
		printf '%s\n' 'Set ROBOT_NAME, e.g. ROBOT_NAME=ada (letters, digits, underscores).' >&2; \
		exit 2; \
	fi

check-map:
	@test -n "$$MAP" || { printf '%s\n' 'Set MAP, e.g. MAP=maps/arena.yaml.' >&2; exit 2; }

check-run:
	@test -n "$$RUN" || { printf '%s\n' 'Set RUN to the folder printed by make record.' >&2; exit 2; }
