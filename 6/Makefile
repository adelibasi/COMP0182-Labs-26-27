# Make settings
SHELL := /bin/bash
.DEFAULT_GOAL := help
.PHONY: mast-head require-robot help build sim localise mapping nav rviz teleop explore check check-hardware test-a-star test-package test-integration

# Configuration: .env, shell, defaults
DEFAULT_VARS := USE_SIM_TIME GUI MAP TURTLEBOT3_MODEL
CONFIG_VARS := ROBOT_NAME ROS_DOMAIN_ID ROS_LOCALHOST_ONLY $(DEFAULT_VARS)
-include .env
$(foreach name,$(CONFIG_VARS),$(eval ENV_ORIGIN_$(name) := $(origin $(name))))
USE_SIM_TIME ?= true
GUI ?= true
MAP ?= maps/house.yaml
TURTLEBOT3_MODEL ?= burger
export $(CONFIG_VARS)

# Mast-head
COLOUR_RESET  := \033[0m
COLOUR_YELLOW := \033[33m
COLOUR_CYAN   := \033[36m
DISPLAY_GOALS := $(if $(MAKECMDGOALS),$(MAKECMDGOALS),help)
source_of = $(strip $(or \
  $(if $(filter command line,$(origin $(1))),command line),\
  $(if $(filter file,$(ENV_ORIGIN_$(1))),.env),\
  $(if $(findstring environment,$(ENV_ORIGIN_$(1))),shell),\
  $(if $(filter $(1),$(DEFAULT_VARS)),Makefile default,unset)))

mast-head:
	@printf '$(COLOUR_CYAN)==>$(COLOUR_RESET) Lab 6: ROS 2 Humble\n'
	@printf '$(COLOUR_CYAN)==>$(COLOUR_RESET) Command: $(COLOUR_YELLOW)%s$(COLOUR_RESET)\n' '$(DISPLAY_GOALS)'
	@printf '\n$(COLOUR_CYAN)ROS variables: $(COLOUR_RESET)\n'
	@args=( $(foreach name,$(CONFIG_VARS),"$(name):$(call source_of,$(name))") ); \
	for spec in "$${args[@]}"; do \
		name=$${spec%%:*}; source=$${spec#*:}; \
		printf '  %s: $(COLOUR_YELLOW)%s$(COLOUR_RESET) (source: %s)\n' "$$name" "$${!name:-<unset>}" "$$source"; \
	done
	@printf '\n'

# Commands
ROS_BASE := source /opt/ros/humble/setup.bash &&
ROS_OVERLAY := $(ROS_BASE) source ros2_ws/install/setup.bash &&
require-robot:
	@[[ "$$ROBOT_NAME" =~ ^tb_[A-Za-z0-9_]+$$ ]] || { printf 'Set ROBOT_NAME to tb_<NAME> (e.g. tb_ada).\n' >&2; exit 2; }

##@ Main commands
help: mast-head ## Show available commands
	@printf 'Usage: $(COLOUR_YELLOW)make <command>$(COLOUR_RESET)\n'
	@awk 'BEGIN { FS = ":.*## " } \
		/^##@ / { printf "\n$(COLOUR_CYAN)%s$(COLOUR_RESET)\n", substr($$0, 5); next } \
		/^[a-zA-Z0-9_-]+:.*## / { printf "  %-17s %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@printf '\nSet ROBOT_NAME to the full namespace (e.g. tb_ada) and ROS_DOMAIN_ID to the demonstrator value.\n'

build: mast-head ## Build the Humble planner packages
	@$(ROS_BASE) mkdir -p ros2_ws && cd ros2_ws && colcon build --base-paths ../packages --symlink-install --packages-up-to lab6_planner

##@ Simulation and navigation
sim: mast-head require-robot ## Start Gazebo in simulation time
	@$(ROS_BASE) ros2 launch ./launch/sim.launch.py "robot_name:=$$ROBOT_NAME" "gui:=$$GUI"

localise: mast-head require-robot ## Start the supplied map and AMCL in simulation
	@$(ROS_BASE) ros2 launch ./launch/amcl.launch.py "robot_name:=$$ROBOT_NAME" "map_file:=$$MAP" "use_sim_time:=true"

mapping: mast-head require-robot check-hardware ## Start physical RTAB-Map; leave AMCL stopped
	@$(ROS_BASE) ros2 launch ./launch/rtab.launch.py "robot_name:=$$ROBOT_NAME"

ifeq ($(USE_SIM_TIME),false)
NAV_CHECK := check-hardware
endif
nav: mast-head require-robot $(NAV_CHECK) ## Start Nav2 with the custom planner bridge
	@$(ROS_OVERLAY) ros2 launch ./launch/nav.launch.py "robot_name:=$$ROBOT_NAME" "use_sim_time:=$$USE_SIM_TIME"

rviz: mast-head require-robot ## Open the map and navigation view
	@$(ROS_BASE) ros2 launch ./launch/rviz.launch.py "robot_name:=$$ROBOT_NAME" "use_sim_time:=$$USE_SIM_TIME" "fixed_frame:=map"

teleop: mast-head require-robot ## Drive robot; stop Nav2 first
	@$(ROS_BASE) exec ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r "cmd_vel:=/$$ROBOT_NAME/cmd_vel"

EXPLORE_DIR := ros2_ws/src/m-explore-ros2
EXPLORE_URL := https://github.com/robo-friends/m-explore-ros2.git
EXPLORE_REV := 326cf8a0b487c34246bb8f3326afbcd69576dc60
explore: mast-head require-robot ## Fetch explore_lite and open the unfinished exercise
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
	@$(ROS_OVERLAY) ros2 launch ./exercises/explore_starter.launch.py "robot_name:=$$ROBOT_NAME" "use_sim_time:=$$USE_SIM_TIME"

##@ Checks
EXPECT ?= stub
check: override EXPECT := route
check: mast-head require-robot test-package test-integration ## Check navigation goals and waypoints with a supplied route.
	@printf 'The A* algorithm was not tested; run make test-a-star.\n'

check-hardware: mast-head require-robot ## Check physical scan, odometry, TF and Twist interface
	@$(ROS_BASE) python3 scripts/check_hardware.py "$$ROBOT_NAME"

test-a-star: mast-head ## Check A* on small costmaps
	@PYTHONPATH=tests:exercises:scripts:$${PYTHONPATH:-} python3 -m unittest test_a_star test_a_star_contract.StudentContractTests -v

test-package: mast-head
	@python3 -m unittest discover -s tests -p test_lab6.py -v

test-integration: mast-head require-robot build
	@$(ROS_OVERLAY) python3 tests/test_ros_integration.py --robot-name "$$ROBOT_NAME" --expect "$(EXPECT)"
