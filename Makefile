# make file inspired by https://roborovsky-racers.github.io/RoborovskyNote/
SHELL := /bin/bash

.PHONY: autoware-build autoware-vehicle autoware-simulator autoware-request-initialpose autoware-request-control  awsim-request-start awsim-request-reset autoware-driver-zenoh \
	simulator rosbag-cleaner clean-rosbags dev ga ga-search ga-status ga-logs ga-stop ga-dashboard ga-parallel ga-joint ga-shared ga-shared-resume ga-shared-stop ga-parallel-resume ga-ipc-clean ga-workers-start ga-parallel-rviz ga-parallel-status ga-parallel-logs ga-parallel-stop \
	dev2 dev3 dev4 ga-ghost4-poc ga-ghost4-poc-stop driver zenoh download rviz2 down down2 down3 down4 ps autoware-bash

# Used by docker-compose.yml for build/eval artifact ownership.
HOST_UID ?= $(shell id -u)
HOST_GID ?= $(shell id -g)
export HOST_UID HOST_GID
GA_CONFIG ?= /aichallenge/ml_workspace/genetic_algorithm/config/experiment_parallel_ros2.yaml
# Stop host shell's ROS_DOMAIN_ID from overriding .env via compose interpolation,
# but still honor an explicit `make foo ROS_DOMAIN_ID=N` command-line override.
unexport ROS_DOMAIN_ID
ifeq ($(origin ROS_DOMAIN_ID),command line)
export ROS_DOMAIN_ID
endif

TIMESTAMP := $(shell date +%Y%m%d-%H%M%S)
LOG_DIR := /output/$(TIMESTAMP)

# autowareのbuildのみ
autoware-build:
	docker compose run -T --rm --no-deps autoware-build

# run autoware for vehicle
autoware-vehicle:
	@echo "Start Autoware for Vehicle"
	LOG_DIR=$(LOG_DIR) RUN_MODE=vehicle docker compose up -d autoware

# run autoware for simulator
autoware-simulator:
	@echo "Start Autoware for AWSIM"
	LOG_DIR=$(LOG_DIR) RUN_MODE=awsim docker compose up -d autoware

# autoware command service use ROS_DOMAIN_ID from .env
autoware-request-initialpose:
	CMD="ros2 service call /set_initial_pose std_srvs/srv/Trigger '{}'" docker compose run --rm --no-deps autoware-command

autoware-request-control:
	CMD="ros2 topic pub -1 /awsim/control_mode_request_topic std_msgs/msg/Bool '{data: true}'" docker compose run --rm --no-deps autoware-command

# awsim admin service use ROS_DOMAIN_ID 0
awsim-request-start:
	CMD="env ROS_DOMAIN_ID=0 ros2 topic pub -1 /admin/awsim/start std_msgs/msg/Bool '{data: true}'" docker compose run --rm --no-deps autoware-command

awsim-request-reset:
	CMD="env ROS_DOMAIN_ID=0 ros2 topic pub -1 /admin/awsim/reset std_msgs/msg/Empty '{}'" docker compose run --rm --no-deps autoware-command

# run simulator (docker compose up -d simulator)
simulator:
	@echo "Start AWSIM (SIM_MODE=$(SIM_MODE))"
	LOG_DIR=$(LOG_DIR) SIM_MODE=$(SIM_MODE) ROS_DOMAIN_ID=0 docker compose up -d simulator

rosbag-cleaner:
	@echo "Start periodic rosbag cleanup"
	docker compose up -d rosbag-cleaner

clean-rosbags:
	docker compose run -T --rm --no-deps --entrypoint /aichallenge/utils/cleanup_rosbags.bash rosbag-cleaner --once

# racing kart (docker compose up -d driver)
driver:
	docker compose up -d driver

# zenoh (docker compose up -d zenoh)
zenoh:
	docker compose up -d zenoh

dev: SIM_MODE := dev
dev: rosbag-cleaner simulator autoware-simulator
	@echo "Start dev simulation (AWSIM + Autoware only; GA search is not started)"
	@echo "To start the GA runner: make ga-search"
	@echo "To stop: make down  (docker compose down --remove-orphans)"

ga:
	GA_EXPERIMENT_MODE=true $(MAKE) dev
	$(MAKE) ga-search

ga-search:
	@ready=false; \
	for attempt in $$(seq 1 60); do \
		if docker compose exec -T autoware bash -lc 'source /opt/ros/humble/setup.bash && source /aichallenge/workspace/install/setup.bash && ROS_DOMAIN_ID=1 ros2 param get /simple_pure_pursuit_node ga_experiment_mode' 2>/dev/null | grep -q 'True'; then \
			ready=true; break; \
		fi; \
		sleep 1; \
	done; \
	$$ready || { echo "Autoware is not running in GA mode. Run: make ga"; exit 1; }
	docker compose up -d --force-recreate ga-runner
	@echo "GA runner started. Follow it with: make ga-logs"

ga-status:
	@docker compose ps ga-runner
	@latest=$$(find aichallenge/ml_workspace/genetic_algorithm/runs -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-); \
	if [ -n "$$latest" ]; then echo "latest_run=$$latest"; fi

ga-logs:
	docker compose logs -f --tail=100 ga-runner

ga-stop:
	docker compose stop ga-runner

ga-dashboard:
	@latest=$$(find aichallenge/ml_workspace/genetic_algorithm/runs -mindepth 1 -maxdepth 1 -type d -name '20*' -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-); \
	test -n "$$latest" || { echo "No GA run found"; exit 2; }; \
	PYTHONPATH=aichallenge/ml_workspace/genetic_algorithm/src python3 -m ga_pure_pursuit.progress_dashboard \
		--run-dir "$$latest" --output "$$latest/dashboard.html"; \
	echo "Open: $$latest/dashboard.html"

ga-ipc-clean:
	@find aichallenge/ml_workspace/genetic_algorithm/ipc \
		-mindepth 3 -maxdepth 3 -type f -name '*.json' -delete

ga-workers-start:
	@mkdir -p output/ga-workers/worker-1/$(TIMESTAMP) output/ga-workers/worker-2/$(TIMESTAMP) output/ga-workers/worker-3/$(TIMESTAMP)
	GA_WORKER_ID=worker-1 GA_WORKER_LOG_DIR=/output/ga-workers/worker-1/$(TIMESTAMP) GA_CONFIG=$(GA_CONFIG) \
		docker compose -f docker-compose.ga-worker.yml -p ga-w1 up -d
	GA_WORKER_ID=worker-2 GA_WORKER_LOG_DIR=/output/ga-workers/worker-2/$(TIMESTAMP) GA_CONFIG=$(GA_CONFIG) \
		docker compose -f docker-compose.ga-worker.yml -p ga-w2 up -d
	GA_WORKER_ID=worker-3 GA_WORKER_LOG_DIR=/output/ga-workers/worker-3/$(TIMESTAMP) GA_CONFIG=$(GA_CONFIG) \
		docker compose -f docker-compose.ga-worker.yml -p ga-w3 up -d

ga-parallel:
	$(MAKE) down
	$(MAKE) ga-ipc-clean
	$(MAKE) rosbag-cleaner
	$(MAKE) ga-workers-start
	GA_CONFIG=/aichallenge/ml_workspace/genetic_algorithm/config/experiment_parallel_ros2.yaml \
		docker compose up -d --force-recreate ga-runner
	@echo "Three-worker GA started. Follow it with: make ga-parallel-status"

ga-joint:
	$(MAKE) down
	$(MAKE) ga-ipc-clean
	$(MAKE) rosbag-cleaner
	$(MAKE) ga-workers-start GA_CONFIG=/aichallenge/ml_workspace/genetic_algorithm/config/experiment_joint_dual_path_ros2.yaml
	GA_CONFIG=/aichallenge/ml_workspace/genetic_algorithm/config/experiment_joint_dual_path_ros2.yaml \
		docker compose up -d --force-recreate ga-runner
	$(MAKE) ga-parallel-rviz
	@echo "Joint Dual Preview + path GA started. Follow it with: make ga-parallel-status"

ga-shared:
	$(MAKE) down
	$(MAKE) rosbag-cleaner
	$(MAKE) ga-ghost4-poc
	@ready=false; \
	for attempt in $$(seq 1 180); do \
		ready=true; \
		for domain in 1 2 3 4; do \
			docker exec ghost-d$$domain-autoware-1 bash -lc \
				"source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=$$domain ros2 service list" \
				2>/dev/null | grep -q '/simple_pure_pursuit_node/ga/set_enabled' || ready=false; \
		done; \
		$$ready && break; \
		sleep 1; \
	done; \
	$$ready || { echo "Shared AWSIM Autoware domains did not become ready"; exit 1; }
	GA_CONFIG=/aichallenge/ml_workspace/genetic_algorithm/config/experiment_shared_ghost4_ros2.yaml \
		docker compose up -d --force-recreate ga-runner
	@echo "Shared-AWSIM four-candidate GA started. Follow it with: make ga-logs"

ga-shared-stop:
	-docker compose stop ga-runner
	$(MAKE) ga-ghost4-poc-stop

ga-shared-resume:
	@test -n "$(RUN_ID)" || { echo "Usage: make ga-shared-resume RUN_ID=YYYYMMDDTHHMMSSZ"; exit 2; }
	@test -d "aichallenge/ml_workspace/genetic_algorithm/runs/$(RUN_ID)" || { echo "Run not found: $(RUN_ID)"; exit 2; }
	@docker compose stop ga-runner
	@for domain in 1 2 3 4; do \
		docker inspect -f '{{.State.Running}}' ghost-d$$domain-autoware-1 2>/dev/null | grep -q true || \
		{ echo "Shared AWSIM infrastructure is not running; run make ga-shared first"; exit 1; }; \
	done
	GA_RESUME_RUN_DIR=/aichallenge/ml_workspace/genetic_algorithm/runs/$(RUN_ID) \
	GA_EVALUATOR_CONFIG=/aichallenge/ml_workspace/genetic_algorithm/config/experiment_shared_ghost4_ros2.yaml \
	GA_PARALLEL_WORKERS=4 docker compose up -d --force-recreate ga-runner
	@echo "Resumed $(RUN_ID) with the shared-AWSIM batch evaluator."

ga-parallel-resume:
	@test -n "$(RUN_ID)" || { echo "Usage: make ga-parallel-resume RUN_ID=YYYYMMDDTHHMMSSZ"; exit 2; }
	@test -d "aichallenge/ml_workspace/genetic_algorithm/runs/$(RUN_ID)" || { echo "Run not found: $(RUN_ID)"; exit 2; }
	$(MAKE) ga-parallel-stop
	$(MAKE) ga-ipc-clean
	$(MAKE) ga-workers-start GA_CONFIG=/aichallenge/ml_workspace/genetic_algorithm/runs/$(RUN_ID)/experiment_resolved.json
	GA_RESUME_RUN_DIR=/aichallenge/ml_workspace/genetic_algorithm/runs/$(RUN_ID) \
		GA_PARALLEL_WORKERS=3 GA_WORKER_IDS=worker-1,worker-2,worker-3 \
		docker compose up -d --force-recreate ga-runner
	$(MAKE) ga-parallel-rviz
	@echo "Resumed $(RUN_ID) with three workers."

ga-parallel-rviz:
	@GA_WORKER_ID=worker-1 docker compose --profile visualization \
		-f docker-compose.ga-worker.yml -p ga-w1 up -d rviz
	@echo "RViz started on worker-1. Stop only RViz with:"
	@echo "  GA_WORKER_ID=worker-1 docker compose --profile visualization -f docker-compose.ga-worker.yml -p ga-w1 stop rviz"

ga-parallel-status:
	@docker compose ps ga-runner rosbag-cleaner
	@GA_WORKER_ID=worker-1 docker compose -f docker-compose.ga-worker.yml -p ga-w1 ps
	@GA_WORKER_ID=worker-2 docker compose -f docker-compose.ga-worker.yml -p ga-w2 ps
	@GA_WORKER_ID=worker-3 docker compose -f docker-compose.ga-worker.yml -p ga-w3 ps
	@latest=$$(find aichallenge/ml_workspace/genetic_algorithm/runs -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-); \
	if [ -n "$$latest" ]; then echo "latest_run=$$latest"; fi

ga-parallel-logs:
	docker compose logs -f --tail=100 ga-runner

ga-parallel-stop:
	-docker compose stop ga-runner
	-GA_WORKER_ID=worker-1 docker compose -f docker-compose.ga-worker.yml -p ga-w1 down --remove-orphans
	-GA_WORKER_ID=worker-2 docker compose -f docker-compose.ga-worker.yml -p ga-w2 down --remove-orphans
	-GA_WORKER_ID=worker-3 docker compose -f docker-compose.ga-worker.yml -p ga-w3 down --remove-orphans

dev2: SIM_MODE := 2p
dev3: SIM_MODE := 3p
dev4: SIM_MODE := 4p
dev2 dev3 dev4: rosbag-cleaner simulator
	@N=$(@:dev%=%); \
	echo "Start $$N-vehicle dev (autoware on ROS_DOMAIN_ID 1..$$N via docker compose -p)"; \
	for p in $$(seq 1 $$N); do LOG_DIR=$(LOG_DIR) ROS_DOMAIN_ID=$$p docker compose -p $$p up -d autoware; done; \
	$(MAKE) awsim-request-start; \
	echo "To Stop: make down"

ga-ghost4-poc:
	@echo "Start one AWSIM with four collision-free vehicles at the D1 pose"
	@mkdir -p output/ga-ghost4-poc/$(TIMESTAMP)
	GA_EXPERIMENT_MODE=true AWSIM_HEADLESS=true SIM_MODE=ghost4 \
		LOG_DIR=/output/ga-ghost4-poc/$(TIMESTAMP) ROS_DOMAIN_ID=0 docker compose up -d simulator
	@for p in $$(seq 1 4); do \
		LOG_DIR=/output/ga-ghost4-poc/$(TIMESTAMP) RUN_MODE=awsim-no-viz \
		GA_EXPERIMENT_MODE=true ROS_DOMAIN_ID=$$p docker compose -p ghost-d$$p up -d autoware; \
	done
	@echo "Four domains are starting. Trigger the common synchronized start with: make awsim-request-start"
	@echo "To stop: make ga-ghost4-poc-stop"

ga-ghost4-poc-stop:
	@for p in $$(seq 1 4); do docker compose -p ghost-d$$p down --remove-orphans; done
	@docker compose stop simulator

# Kept for backward compatibility; `make down` already cleans all projects.
down2 down3 down4: down

eval:
	@echo "Start evaluation simulation (AWSIM + Autoware)"
	docker compose up -d rosbag-cleaner autoware-simulator-evaluation
	@echo "To stop: make down  (docker compose down --remove-orphans)"

# remote operation (docker compose up -d rviz2)
rviz2:
	docker compose stop rviz2
	docker compose up -d rviz2

# driver + autoware + zenoh
autoware-driver-zenoh:
	RUN_MODE=vehicle docker compose up -d driver autoware
	sleep 15
	docker compose up -d zenoh

down:
	@GA_WORKER_ID=worker-1 docker compose -f docker-compose.ga-worker.yml -p ga-w1 down --remove-orphans 2>/dev/null || true
	@GA_WORKER_ID=worker-2 docker compose -f docker-compose.ga-worker.yml -p ga-w2 down --remove-orphans 2>/dev/null || true
	@GA_WORKER_ID=worker-3 docker compose -f docker-compose.ga-worker.yml -p ga-w3 down --remove-orphans 2>/dev/null || true
	@for p in 1 2 3 4; do docker compose -p $$p down --remove-orphans; done
	@for p in 1 2 3 4; do docker compose -p ghost-d$$p down --remove-orphans; done
	@docker compose down --remove-orphans

down_all:
	sudo docker ps -aq | xargs -r sudo docker rm -f

ps:
	@docker compose ps
	@for p in 1 2 3 4; do \
		out=$$(docker compose -p $$p ps --format '{{.Name}}\t{{.Service}}\t{{.Status}}' 2>/dev/null); \
		if [ -n "$$out" ]; then \
			echo "--- project=$$p ---"; \
			echo "$$out"; \
		fi; \
	done

autoware-bash:
	@if [ -z "$(VEHICLE_NUM)" ]; then \
		docker compose exec autoware bash; \
	else \
		docker compose -p $(VEHICLE_NUM) exec autoware bash; \
	fi

# Download submission data by asking for credentials interactively
# Usage:
#   make download [SUBMISSION_ID=<id>]
# Usage (Only Admins):
#   make download [USER_ID=<id>] [SUBMISSION_ID=<id>]
download:
	@if [ -n "$(USER_ID)" ]; then \
		if [ -n "$(SUBMISSION_ID)" ]; then \
			vehicle/download_submission.sh --output aichallenge/workspace/src/ --user-id $(USER_ID) --submission-id $(SUBMISSION_ID); \
		else \
			vehicle/download_submission.sh --output aichallenge/workspace/src/ --user-id $(USER_ID); \
		fi; \
	else \
		if [ -n "$(SUBMISSION_ID)" ]; then \
			vehicle/download_submission.sh --output aichallenge/workspace/src/ --submission-id $(SUBMISSION_ID); \
		else \
			vehicle/download_submission.sh --output aichallenge/workspace/src/; \
		fi; \
	fi
