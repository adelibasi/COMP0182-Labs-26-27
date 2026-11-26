-include .env

# record caller origins before assigning defaults
$(foreach name,ROBOT_NAME ROS_DOMAIN_ID USE_SIM_TIME ROS_LOCALHOST_ONLY GUI MAP TURTLEBOT3_MODEL CAMERA_TOPIC,\
  $(eval ENV_ORIGIN_$(name) := $(origin $(name))))

##@ Variables
## Full robot namespace such as tb_ada
export ROBOT_NAME ?=
## DDS domain assigned by the demonstrator
export ROS_DOMAIN_ID ?=
## Use the simulation clock
export USE_SIM_TIME ?= true
## Keep ROS traffic on this machine
export ROS_LOCALHOST_ONLY ?= 1
## Open the simulator GUI
export GUI ?= true
## Map YAML to load
export MAP ?= maps/arena.yaml
## TurtleBot3 model
export TURTLEBOT3_MODEL ?= burger
## Override the decoded image topic for the camera check
export CAMERA_TOPIC ?=

# make settings
SHELL := /bin/bash
.DEFAULT_GOAL := help

# configuration: .env, shell, defaults
DEFAULT_VARS := USE_SIM_TIME ROS_LOCALHOST_ONLY GUI MAP TURTLEBOT3_MODEL
CONFIG_VARS := ROBOT_NAME ROS_DOMAIN_ID $(DEFAULT_VARS) CAMERA_TOPIC

# mast-head
COLOUR_RESET  := \033[0m
COLOUR_YELLOW := \033[33m
COLOUR_CYAN   := \033[36m
DISPLAY_GOALS := $(if $(MAKECMDGOALS),$(MAKECMDGOALS),help)
source_of = $(strip $(or \
  $(if $(filter command line,$(origin $(1))),command line),\
  $(if $(filter file,$(ENV_ORIGIN_$(1))),.env),\
  $(if $(findstring environment,$(ENV_ORIGIN_$(1))),shell),\
  $(if $(filter $(1),$(DEFAULT_VARS)),Makefile default,unset)))

# commands
PYTHON ?= bash scripts/python.sh
ROS_BASE := source /opt/ros/humble/setup.bash && if [[ -f "$$HOME/turtlebot3_ws/install/local_setup.bash" ]]; then source "$$HOME/turtlebot3_ws/install/local_setup.bash"; fi &&
ROS_OVERLAY := $(ROS_BASE) source ros2_ws/install/setup.bash &&
EXPLORE_DIR := ros2_ws/src/m-explore-ros2
EXPLORE_URL := https://github.com/robo-friends/m-explore-ros2.git
EXPLORE_REV := 326cf8a0b487c34246bb8f3326afbcd69576dc60

.PHONY: mast-head help require-robot require-sim build sim localise mapping nav \
	rviz teleop pose explore decode mission check check-hardware

mast-head:
	@printf '$(COLOUR_CYAN)==>$(COLOUR_RESET) Lab 7: ArUco Markers and CV\n'
	@printf '$(COLOUR_CYAN)==>$(COLOUR_RESET) Command: $(COLOUR_YELLOW)%s$(COLOUR_RESET)\n' '$(DISPLAY_GOALS)'
	@printf '\n$(COLOUR_CYAN)ROS variables: $(COLOUR_RESET)\n'
	@args=( $(foreach name,$(CONFIG_VARS),"$(name):$(call source_of,$(name))") ); \
	for spec in "$${args[@]}"; do \
		name=$${spec%%:*}; source=$${spec#*:}; \
		printf '  %s: $(COLOUR_YELLOW)%s$(COLOUR_RESET) (source: %s)\n' "$$name" "$${!name:-<unset>}" "$$source"; \
	done
	@printf '\n'

##@ Main commands
help: mast-head ## Show available commands
	@printf '\nUsage: $(COLOUR_YELLOW)make <command> [VARIABLE=value]$(COLOUR_RESET)\n'
	@awk '\
		function row(name, text) { printf "  %-17s %s\n", name, text } \
		FNR == 1 { note = "" } \
		{ sub(/\r$$/, "") } \
		/^##@ / { printf "\n$(COLOUR_CYAN)%s$(COLOUR_RESET)\n", substr($$0, 5); note = ""; next } \
		/^## / { note = substr($$0, 4); next } \
		/^ *((override|export|private)[ \t]+)*[a-zA-Z0-9_.-]+[ \t]*(:|::|:::|[!?+])?=/ { \
			if (note != "") { \
				name = $$0; sub(/^ *((override|export|private)[ \t]+)*/, "", name); \
				sub(/[^a-zA-Z0-9_.-].*$$/, "", name); row(name, note) \
			} \
			note = ""; next \
		} \
		{ note = "" } \
		/^ *[a-zA-Z0-9_.-]+[ \t]*:[^#]* ## / { \
			match($$0, /[a-zA-Z0-9_.-]+/); name = substr($$0, RSTART, RLENGTH); \
			match($$0, / ## /); row(name, substr($$0, RSTART + RLENGTH)) \
		}' $(MAKEFILE_LIST)

build: mast-head ## Build the Humble planner and mission packages
	@$(ROS_BASE) mkdir -p ros2_ws && cd ros2_ws && colcon build --base-paths ../packages --symlink-install --packages-up-to planner mission

##@ Simulation and navigation
sim: ## Start Gazebo in simulation time
	@$(ROS_BASE) exec ros2 launch ./launch/sim.launch.py "robot_name:=$$ROBOT_NAME" "gui:=$$GUI"

localise: ## Start the supplied map and AMCL
	@$(ROS_BASE) exec ros2 launch ./launch/amcl.launch.py "robot_name:=$$ROBOT_NAME" "map_file:=$$MAP" "use_sim_time:=$$USE_SIM_TIME" "seed_start:=$$USE_SIM_TIME"

mapping: ## Start RTAB-Map with AMCL stopped
	@$(ROS_BASE) exec ros2 launch ./launch/rtab.launch.py "robot_name:=$$ROBOT_NAME" "use_sim_time:=$$USE_SIM_TIME"

nav: ## Start Nav2 with the custom planner bridge
	@$(ROS_OVERLAY) exec ros2 launch ./launch/nav.launch.py "robot_name:=$$ROBOT_NAME" "use_sim_time:=$$USE_SIM_TIME"

rviz: ## Open the map and navigation view
	@$(ROS_BASE) exec ros2 launch ./launch/rviz.launch.py "robot_name:=$$ROBOT_NAME" "use_sim_time:=$$USE_SIM_TIME" "fixed_frame:=map"

teleop: ## Drive the robot after stopping Nav2
	@$(ROS_BASE) exec ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r "cmd_vel:=/$$ROBOT_NAME/cmd_vel"

pose: ## Show the robot's position and heading in the map frame
	@$(ROS_BASE) exec ros2 run tf2_ros tf2_echo map "$$ROBOT_NAME/base_footprint"

explore: ## Fetch explore_lite and run the launch exercise
	@set -e; \
	if [ ! -d "$(EXPLORE_DIR)/.git" ]; then \
		mkdir -p "$(EXPLORE_DIR)"; \
		git -C "$(EXPLORE_DIR)" init -q; \
		git -C "$(EXPLORE_DIR)" remote add origin "$(EXPLORE_URL)"; \
	fi; \
	if [ "$$(git -C "$(EXPLORE_DIR)" rev-parse HEAD 2>/dev/null)" != "$(EXPLORE_REV)" ]; then \
		git -C "$(EXPLORE_DIR)" fetch --depth 1 origin "$(EXPLORE_REV)"; \
		git -C "$(EXPLORE_DIR)" checkout --detach FETCH_HEAD; \
	fi
	@$(ROS_BASE) cd ros2_ws && colcon build --symlink-install --packages-up-to explore_lite
	@$(ROS_OVERLAY) exec ros2 launch ./exercises/explore_starter.launch.py "robot_name:=$$ROBOT_NAME" "use_sim_time:=$$USE_SIM_TIME"

##@ Marker reading
decode: ## Decode H.264 and publish ArUco frames for RViz
	@$(ROS_BASE) exec ros2 launch ./packages/camera_decoder/decode_camera.launch.py "robot_name:=$$ROBOT_NAME" "use_sim_time:=$$USE_SIM_TIME"

mission: ## Run the autonomous marker mission with recorded poses
	@$(ROS_OVERLAY) exec ros2 run mission autonomous_search.py --ros-args \
		-r "__ns:=/$$ROBOT_NAME" -p "robot_name:=$$ROBOT_NAME" -p "use_sim_time:=$$USE_SIM_TIME"

##@ Checks
check-hardware: ## Check the configured TurtleBot is visible
	@$(ROS_BASE) python3 scripts/check_hardware.py "$$ROBOT_NAME"

check: ## Run the Lab 7 preflight for SIM or REAL
	@ROS_LOCALHOST_ONLY=1 $(PYTHON) scripts/aruco.py --robot-name "$$ROBOT_NAME" \
		--use-sim-time "$$USE_SIM_TIME" --image-topic "$$CAMERA_TOPIC" --check

require-robot:
	@[[ "$$ROBOT_NAME" =~ ^tb_[A-Za-z0-9_]+$$ ]] || { printf 'Set ROBOT_NAME=tb_<NAME> in .env.\n' >&2; exit 2; }
	@[[ "$$ROS_DOMAIN_ID" =~ ^[0-9]+$$ ]] && (( 10#$$ROS_DOMAIN_ID <= 232 )) || { printf 'Set ROS_DOMAIN_ID to the assigned domain (0-232) in .env.\n' >&2; exit 2; }

require-sim:
	@[[ "$$USE_SIM_TIME" == true && "$$ROS_LOCALHOST_ONLY" == 1 ]] || { printf 'For SIM, set USE_SIM_TIME=true and ROS_LOCALHOST_ONLY=1 in .env.\n' >&2; exit 2; }

sim localise mapping nav rviz teleop pose explore decode mission check-hardware check: mast-head require-robot
sim: require-sim

ifeq ($(USE_SIM_TIME),true)
decode check mission: require-sim
else ifeq ($(USE_SIM_TIME),false)
mapping nav mission: check-hardware
endif
