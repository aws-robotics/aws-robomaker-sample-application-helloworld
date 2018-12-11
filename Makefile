.PHONY: all setup build build_robot build_simulation bundle bundle_robot bundle_simulation clean clean_robot_build clean_robot_bundle clean_simulation_build clean_simulation_bundle
.DEFAULT_GOAL := all

all: bundle

# This forces each step in 'ci' to run in serial, but each step will run all of its commands in parallel
ci:
	$(MAKE) setup
	$(MAKE) build
	$(MAKE) bundle

setup:
	scripts/setup.sh

build: build_robot build_simulation

build_robot:
	scripts/build.sh ./robot_ws

build_simulation:
	scripts/build.sh ./simulation_ws

bundle: bundle_robot bundle_simulation

bundle_robot: build_robot
	scripts/bundle.sh ./robot_ws

bundle_simulation: build_simulation
	scripts/bundle.sh ./simulation_ws

clean: clean_robot_build clean_robot_bundle clean_simulation_build clean_simulation_bundle

clean_robot_build:
	rm -rf ./robot_ws/build ./robot_ws/install

clean_robot_bundle:
	rm -rf ./robot_ws/bundle

clean_simulation_build:
	rm -rf ./simulation_ws/build ./simulation_ws/install

clean_simulation_bundle:
	rm -rf ./simulation_ws/bundle
