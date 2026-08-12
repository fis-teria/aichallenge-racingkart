# make file inspired by https://roborovsky-racers.github.io/RoborovskyNote/
SHELL := /bin/bash

# Gate2 is an authority-bearing dispatch path.  Reject shell/make startup
# injection before any Gate2 prerequisite or recipe is evaluated, and never
# export those controls to a recursive dispatch.  `MAKEFLAGS` is Make's own
# state; only the harmless no-print-directory form is accepted here.
GATE2_PROTECTED_GOALS := gate2 gate2-wrapper-authorization gate2-live-authority-flags gate2-current-artifact-admission planner-pp-control-smoke planner-pp-control-smoke-eval
GATE2_PROTECTED_GOAL_REQUESTED := $(filter $(GATE2_PROTECTED_GOALS),$(MAKECMDGOALS))
ifneq ($(strip $(GATE2_PROTECTED_GOAL_REQUESTED)),)
# Keep the shell and recursive-make entry points owned by this file.  A
# command-line value must be rejected even when it happens to equal the
# reviewed default; an untrusted environment value is rejected unless it is
# the ordinary host `/bin/bash` setting which GNU Make otherwise ignores.
ifneq ($(origin SHELL),file)
$(error PRECONDITION_NOT_MET: gate2_make_variable_override:SHELL)
endif
GATE2_RAW_SHELL := $(strip $(shell /usr/bin/printenv SHELL 2>/dev/null))
ifneq ($(GATE2_RAW_SHELL),)
ifneq ($(GATE2_RAW_SHELL),/bin/bash)
$(error PRECONDITION_NOT_MET: gate2_make_environment_override:SHELL)
endif
endif
ifneq ($(filter command line environment override command line override environment override,$(origin MAKE)),)
$(error PRECONDITION_NOT_MET: gate2_make_variable_override:MAKE)
endif
# These values select the target-specific runtime and command service.  A
# top-level command-line/environment value would override the target-specific
# reviewed assignments.  The existing planner recipe intentionally forwards
# its fixed values to one recursive Make invocation, so that inner dispatch is
# allowed to carry those already-reviewed values.
GATE2_DISPATCH_OVERRIDE_VARS := AUTOWARE_SERVICE AUTOWARE_COMMAND_SERVICE AUTOWARE_COMMAND_MODE AUTOWARE_RUNTIME_IMAGE AWSIM_START_TARGET AUTOWARE_RUN_MODE AUTOSTART_DEBUG_VISUALIZATION CONTROL_METHOD RUN_KIND
GATE2_DISPATCH_OVERRIDES := $(foreach var,$(GATE2_DISPATCH_OVERRIDE_VARS),$(if $(filter command line environment override command line override environment override,$(origin $(var))),$(var)))
ifeq ($(MAKELEVEL),0)
ifneq ($(strip $(GATE2_DISPATCH_OVERRIDES)),)
$(error PRECONDITION_NOT_MET: gate2_dispatch_variable_override:$(strip $(GATE2_DISPATCH_OVERRIDES)))
endif
endif
ifneq ($(strip $(BASH_ENV)),)
$(error PRECONDITION_NOT_MET: gate2_make_environment_injection:BASH_ENV)
endif
ifneq ($(strip $(ENV)),)
$(error PRECONDITION_NOT_MET: gate2_make_environment_injection:ENV)
endif
ifneq ($(strip $(SHELLOPTS)),)
$(error PRECONDITION_NOT_MET: gate2_make_environment_injection:SHELLOPTS)
endif
ifneq ($(strip $(MAKEFILES)),)
$(error PRECONDITION_NOT_MET: gate2_make_environment_injection:MAKEFILES)
endif
GATE2_RAW_GNUMAKEFLAGS := $(strip $(shell /usr/bin/printenv GNUMAKEFLAGS 2>/dev/null))
ifneq ($(strip $(GATE2_RAW_GNUMAKEFLAGS)),)
$(error PRECONDITION_NOT_MET: gate2_make_environment_injection:GNUMAKEFLAGS)
endif
# Recursive make may combine harmless short flags (for example `nw` from
# `make -n`).  Reject every other make flag before any authority-bearing
# prerequisite or recipe is evaluated.
GATE2_MAKEFLAGS_UNSAFE := $(foreach flag,$(filter-out --no-print-directory --,$(strip $(MAKEFLAGS))),$(if $(subst n,,$(subst s,,$(subst w,,$(flag)))),$(flag)))
ifneq ($(strip $(GATE2_MAKEFLAGS_UNSAFE)),)
$(error PRECONDITION_NOT_MET: gate2_makeflags_invalid)
endif
unexport BASH_ENV ENV SHELLOPTS MAKEFILES GNUMAKEFLAGS MAKEOVERRIDES MFLAGS
export RUN_ID AIC_TEST_GATE2_REVIEWED_EXECUTION_ID AIC_TEST_GATE2_REVIEW_ATTEMPT_ID AIC_TEST_GATE2_REVIEW_LAUNCH_SPEC_SHA256 AIC_TEST_GATE2_CAPABILITY_PATH
endif

.PHONY: autoware-build autoware-vehicle autoware-simulator autoware-command-mode-run autoware-command-mode-exec autoware-request-initialpose autoware-request-control awsim-request-start awsim-request-start-run awsim-request-start-exec awsim-request-start-and-watch-d1 awsim-request-reset autoware-driver-zenoh \
	capture-run-fingerprint verify-run-fingerprint simulator dev dev2 dev3 dev4 gate1 gate2 gate3 planner-pp-control-smoke planner-pp-control-smoke-eval gate2-live-authority-flags gate2-wrapper-authorization gate2-current-artifact-admission state-lattice-test-only-observe state-lattice-test-only-replay driver zenoh download rviz2 down down2 down3 down4 ps autoware-bash

# Used by docker-compose.yml for build/eval artifact ownership.
HOST_UID ?= $(shell id -u)
HOST_GID ?= $(shell id -g)
CONTROL_METHOD ?= pure_pursuit_mpc_horizon
PP_CORE_EXACT_SNAPSHOT_ENABLED ?= false
OVERTAKE_TRAJECTORY_BACKEND ?= current
AWSIM_READY_DOMAINS ?= 1
ROSBAG ?= false
RACE_ARM_ON_VEHICLE_STATE ?= Start
AUTOWARE_RUN_MODE ?= awsim
AUTOSTART_DEBUG_VISUALIZATION ?= true
AUTOWARE_SERVICE ?= autoware
AUTOWARE_COMMAND_SERVICE ?= autoware-command
AUTOWARE_COMMAND_MODE ?= run
AUTOWARE_RUNTIME_IMAGE ?= aichallenge-2025-dev
D1_STALL_TIMEOUT_SEC ?= 15
D1_STALL_ENTER_SPEED_MPS ?= 0.05
D1_STALL_EXIT_SPEED_MPS ?= 0.10
D1_VELOCITY_FRESHNESS_SEC ?= 1.0
D1_VELOCITY_EVIDENCE_FAILURE_SEC ?= 15
DEV_AUTO_START ?= true
TEST_ONLY_DURATION_SEC ?= 12
TEST_ONLY_INPUT_ODOM ?= /localization/kinematic_state
# Empty means the ordinary single-vehicle compose project. For dev2/dev3,
# pass TEST_ONLY_COMPOSE_PROJECT=1 (or 2/3) and the matching ROS domain.
TEST_ONLY_COMPOSE_PROJECT ?=
TEST_ONLY_ROS_DOMAIN_ID ?= 1
TEST_ONLY_REPLAY_MODE ?= baseline
TEST_ONLY_REPLAY_ROS_DOMAIN_ID ?= 229
TEST_ONLY_REPLAY_TIMEOUT_SEC ?= 45
TEST_ONLY_REPLAY_SOURCE_BAG ?=
override DEV_AUTO_START_EFFECTIVE_MODE := sync
export HOST_UID HOST_GID ROSBAG RACE_ARM_ON_VEHICLE_STATE
export PP_CORE_EXACT_SNAPSHOT_ENABLED OVERTAKE_TRAJECTORY_BACKEND
OUTPUT_HOST_ROOT ?= ./output
OUTPUT_ROOT ?= /output
export OUTPUT_HOST_ROOT OUTPUT_ROOT
# Stop host shell's ROS_DOMAIN_ID from overriding .env via compose interpolation,
# but still honor an explicit `make foo ROS_DOMAIN_ID=N` command-line override.
unexport ROS_DOMAIN_ID
ifeq ($(origin ROS_DOMAIN_ID),command line)
export ROS_DOMAIN_ID
endif

TIMESTAMP := $(shell date +%Y%m%d-%H%M%S)
RUN_ID ?= $(TIMESTAMP)
RUN_KIND ?= $(firstword $(MAKECMDGOALS))
LOG_DIR := $(patsubst %/,%,$(OUTPUT_ROOT))/$(RUN_ID)
RUN_HOST_DIR := $(patsubst %/,%,$(OUTPUT_HOST_ROOT))/$(RUN_ID)

.NOTPARALLEL: dev dev2 dev3 dev4 gate1 gate2 gate3

capture-run-fingerprint:
	@RUN_KIND="$(RUN_KIND)" \
	SIM_MODE="$(SIM_MODE)" \
	CONTROL_METHOD="$(CONTROL_METHOD)" \
	PP_CORE_EXACT_SNAPSHOT_ENABLED="$(PP_CORE_EXACT_SNAPSHOT_ENABLED)" \
	OVERTAKE_TRAJECTORY_BACKEND="$(OVERTAKE_TRAJECTORY_BACKEND)" \
	PLANNER_PP_CONTROL_SMOKE_LIVE_SPATIAL="$(PLANNER_PP_CONTROL_SMOKE_LIVE_SPATIAL)" \
	STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED="$(STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED)" \
	STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED="$(STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED)" \
	STATE_LATTICE_V2_PRODUCER_INSTANCE_ID="$(STATE_LATTICE_V2_PRODUCER_INSTANCE_ID)" \
	STATE_LATTICE_V2_PP_PRODUCER_INSTANCE_ID="$(STATE_LATTICE_V2_PP_PRODUCER_INSTANCE_ID)" \
	STATE_LATTICE_V2_SESSION_ID="$(STATE_LATTICE_V2_SESSION_ID)" \
	STATE_LATTICE_V4_POC_COMMAND_ACTIVATION_ENABLED="$(STATE_LATTICE_V4_POC_COMMAND_ACTIVATION_ENABLED)" \
	AIC_TEST_GATE2_REVIEW_ANCHOR_PATH="$(AIC_TEST_GATE2_REVIEW_ANCHOR_PATH)" \
	AIC_TEST_GATE2_REVIEW_ANCHOR_SHA256="$(AIC_TEST_GATE2_REVIEW_ANCHOR_SHA256)" \
	AIC_TEST_GATE2_REVIEW_STATE_PATH="$(AIC_TEST_GATE2_REVIEW_STATE_PATH)" \
	AIC_TEST_GATE2_REVIEW_STATE_SHA256="$(AIC_TEST_GATE2_REVIEW_STATE_SHA256)" \
	AIC_TEST_GATE2_REVIEW_RESULT_PATH="$(AIC_TEST_GATE2_REVIEW_RESULT_PATH)" \
	AIC_TEST_GATE2_REVIEW_RESULT_SHA256="$(AIC_TEST_GATE2_REVIEW_RESULT_SHA256)" \
	AIC_TEST_GATE2_REVIEWED_EXECUTION_ID="$(AIC_TEST_GATE2_REVIEWED_EXECUTION_ID)" \
	AIC_TEST_GATE2_REVIEW_ATTEMPT_ID="$(AIC_TEST_GATE2_REVIEW_ATTEMPT_ID)" \
	AIC_TEST_GATE2_REVIEWED_HANDOFF_PATH="$(AIC_TEST_GATE2_REVIEWED_HANDOFF_PATH)" \
	AIC_TEST_GATE2_REVIEWED_HANDOFF_SHA256="$(AIC_TEST_GATE2_REVIEWED_HANDOFF_SHA256)" \
	AIC_TEST_GATE2_REVIEW_PACKET_PATH="$(AIC_TEST_GATE2_REVIEW_PACKET_PATH)" \
	AIC_TEST_GATE2_REVIEW_PACKET_SHA256="$(AIC_TEST_GATE2_REVIEW_PACKET_SHA256)" \
	AIC_TEST_GATE2_REVIEW_BUNDLE_PATH="$(AIC_TEST_GATE2_REVIEW_BUNDLE_PATH)" \
	AIC_TEST_GATE2_REVIEW_BUNDLE_SHA256="$(AIC_TEST_GATE2_REVIEW_BUNDLE_SHA256)" \
	AIC_TEST_GATE2_REVIEW_IMAGE_ID="$(AIC_TEST_GATE2_REVIEW_IMAGE_ID)" \
	AIC_TEST_GATE2_REVIEW_LAUNCH_SPEC_SHA256="$(AIC_TEST_GATE2_REVIEW_LAUNCH_SPEC_SHA256)" \
	AIC_TEST_GATE2_WRAPPER_TOKEN="$(AIC_TEST_GATE2_WRAPPER_TOKEN)" \
	AIC_TEST_GATE2_CAPABILITY_PATH="$(AIC_TEST_GATE2_CAPABILITY_PATH)" \
	STATE_LATTICE_EXACT_SPATIAL_FOLLOW_SHADOW_ENABLED="$(if $(and $(filter planner-pp-control-smoke,$(RUN_KIND)),$(filter true,$(PLANNER_PP_CONTROL_SMOKE_LIVE_SPATIAL))),false,true)" \
	ROSBAG="$(ROSBAG)" \
	AWSIM_START_MODE="$(AWSIM_START_MODE)" \
	AWSIM_START_COUNT_SECONDS="$(AWSIM_START_COUNT_SECONDS)" \
	AWSIM_VEHICLES="$(AWSIM_VEHICLES)" \
	AWSIM_LAPS="$(AWSIM_LAPS)" \
	AWSIM_TIMEOUT="$(AWSIM_TIMEOUT)" \
	AWSIM_EXTRA_ARGS="$(AWSIM_EXTRA_ARGS)" \
	GATE_EXTRA_ARGS="$(GATE_EXTRA_ARGS)" \
	RACE_ARM_ON_VEHICLE_STATE="$(RACE_ARM_ON_VEHICLE_STATE)" \
	AUTOWARE_RUN_MODE="$(AUTOWARE_RUN_MODE)" \
	AUTOSTART_DEBUG_VISUALIZATION="$(AUTOSTART_DEBUG_VISUALIZATION)" \
	AUTOWARE_RUNTIME_IMAGE="$(AUTOWARE_RUNTIME_IMAGE)" \
	AIC_EXPECTED_AUTOWARE_RUNTIME_IMAGE_ID="$(AIC_EXPECTED_AUTOWARE_RUNTIME_IMAGE_ID)" \
	D1_STALL_TIMEOUT_SEC="$(D1_STALL_TIMEOUT_SEC)" \
	D1_STALL_ENTER_SPEED_MPS="$(D1_STALL_ENTER_SPEED_MPS)" \
	D1_STALL_EXIT_SPEED_MPS="$(D1_STALL_EXIT_SPEED_MPS)" \
	D1_VELOCITY_FRESHNESS_SEC="$(D1_VELOCITY_FRESHNESS_SEC)" \
	D1_VELOCITY_EVIDENCE_FAILURE_SEC="$(D1_VELOCITY_EVIDENCE_FAILURE_SEC)" \
	DEV_AUTO_START="$(DEV_AUTO_START)" \
	DEV_AUTO_START_EFFECTIVE_MODE="$(DEV_AUTO_START_EFFECTIVE_MODE)" \
	AWSIM_READY_DOMAINS="$(AWSIM_READY_DOMAINS)" \
	RUN_GATE_SCENARIO="$(RUN_GATE_SCENARIO)" \
	RUN_GATE_ARG="$(RUN_GATE_ARG)" \
	OUTPUT_ROOT="$(OUTPUT_ROOT)" \
	OUTPUT_HOST_ROOT="$(OUTPUT_HOST_ROOT)" \
		python3 aichallenge/capture_run_fingerprint.py \
			--repo-root "$(CURDIR)" \
			--output-root "$(OUTPUT_HOST_ROOT)" \
			--run-dir "$(RUN_HOST_DIR)" \
			--prepare-domains "$(AWSIM_READY_DOMAINS)"

verify-run-fingerprint:
	@if [ -z "$(VERIFY_RUN_ID)" ]; then \
		echo "VERIFY_RUN_ID is required" >&2; \
		exit 2; \
	fi
	@python3 aichallenge/capture_run_fingerprint.py \
		--repo-root "$(CURDIR)" \
		--output-root "$(OUTPUT_HOST_ROOT)" \
		--run-dir "$(patsubst %/,%,$(OUTPUT_HOST_ROOT))/$(VERIFY_RUN_ID)" \
		--verify-postrun

# Observe against an already-running Autoware domain. The observer is hard-wired
# non-live, has no Mux, and writes only below this run's output directory.
state-lattice-test-only-observe:
	@if [ -z "$(RUN_ID)" ]; then \
		echo "RUN_ID is required (use the running dev/gate artifact ID)" >&2; \
		exit 2; \
	fi
	@compose=(docker compose); \
	if [ -n "$(TEST_ONLY_COMPOSE_PROJECT)" ]; then \
		compose+=( -p "$(TEST_ONLY_COMPOSE_PROJECT)" ); \
	fi; \
	CMD="env ROS_DOMAIN_ID=$(TEST_ONLY_ROS_DOMAIN_ID) TEST_ONLY_RUN_ID=$(RUN_ID) TEST_ONLY_ARTIFACT_DIR=$(LOG_DIR)/d$(TEST_ONLY_ROS_DOMAIN_ID)/test_only_state_lattice_observer TEST_ONLY_DURATION_SEC=$(TEST_ONLY_DURATION_SEC) TEST_ONLY_INPUT_ODOM=$(TEST_ONLY_INPUT_ODOM) bash /aichallenge/run_state_lattice_test_only_observer.bash" \
		"$${compose[@]}" run --rm --no-deps autoware-command

# Replays only an explicit bag allowlist into a localhost-only, private ROS graph.
# This is counterfactual production-algorithm evidence, never official authority.
state-lattice-test-only-replay:
	@if [ -z "$(RUN_ID)" ]; then \
		echo "RUN_ID is required" >&2; \
		exit 2; \
	fi
	@if [ -z "$(TEST_ONLY_REPLAY_SOURCE_BAG)" ]; then \
		echo "TEST_ONLY_REPLAY_SOURCE_BAG is required (absolute /output/... bag path)" >&2; \
		exit 2; \
	fi
	@compose=(docker compose); \
	if [ -n "$(TEST_ONLY_COMPOSE_PROJECT)" ]; then \
		compose+=( -p "$(TEST_ONLY_COMPOSE_PROJECT)" ); \
	fi; \
	CMD="env AWSIM_READY_DOMAINS=$(AWSIM_READY_DOMAINS) TEST_ONLY_REPLAY_RUN_ID=$(RUN_ID) TEST_ONLY_REPLAY_MODE=$(TEST_ONLY_REPLAY_MODE) TEST_ONLY_REPLAY_ROS_DOMAIN_ID=$(TEST_ONLY_REPLAY_ROS_DOMAIN_ID) TEST_ONLY_REPLAY_TIMEOUT_SEC=$(TEST_ONLY_REPLAY_TIMEOUT_SEC) TEST_ONLY_REPLAY_SOURCE_BAG=$(TEST_ONLY_REPLAY_SOURCE_BAG) TEST_ONLY_REPLAY_ARTIFACT_DIR=$(LOG_DIR)/offline_replay/$(TEST_ONLY_REPLAY_MODE) bash /aichallenge/run_state_lattice_pp_mux_offline_replay.bash" \
		"$${compose[@]}" run --rm --no-deps autoware-command

# autowareのbuildのみ
autoware-build:
	docker compose run -T --rm --no-deps autoware-build

# run autoware for vehicle
autoware-vehicle:
	@echo "Start Autoware for Vehicle"
	LOG_DIR=$(LOG_DIR) RUN_MODE=vehicle docker compose up -d autoware

# run autoware for simulator
autoware-simulator: capture-run-fingerprint
	@echo "Start Autoware for AWSIM"
	LOG_DIR=$(LOG_DIR) RUN_MODE="$(AUTOWARE_RUN_MODE)" \
	AUTOSTART_DEBUG_VISUALIZATION="$(AUTOSTART_DEBUG_VISUALIZATION)" \
	RUN_KIND="$(RUN_KIND)" \
	PLANNER_PP_CONTROL_SMOKE_LIVE_SPATIAL="$(PLANNER_PP_CONTROL_SMOKE_LIVE_SPATIAL)" \
	STATE_LATTICE_V4_POC_COMMAND_ACTIVATION_ENABLED="$(STATE_LATTICE_V4_POC_COMMAND_ACTIVATION_ENABLED)" \
	docker compose up -d "$(AUTOWARE_SERVICE)"
	@$(MAKE) autoware-command-mode-$(AUTOWARE_COMMAND_MODE) \
		RUN_ID="$(RUN_ID)" OUTPUT_HOST_ROOT="$(OUTPUT_HOST_ROOT)" \
		AUTOWARE_SERVICE="$(AUTOWARE_SERVICE)" \
		AUTOWARE_COMMAND_SERVICE="$(AUTOWARE_COMMAND_SERVICE)" \
		AUTOWARE_RUNTIME_IMAGE="$(AUTOWARE_RUNTIME_IMAGE)"

autoware-command-mode-run:
	@:

autoware-command-mode-exec:
	docker compose up -d "$(AUTOWARE_COMMAND_SERVICE)"
	@AUTOWARE_RUNTIME_IMAGE="$(AUTOWARE_RUNTIME_IMAGE)" \
		python3 aichallenge/capture_run_fingerprint.py \
			--repo-root "$(CURDIR)" --output-root "$(OUTPUT_HOST_ROOT)" \
			--run-dir "$(RUN_HOST_DIR)" \
			--attest-running-service "$(AUTOWARE_SERVICE)" \
			--attest-running-service "$(AUTOWARE_COMMAND_SERVICE)"

autoware-command-mode-%:
	@echo "invalid AUTOWARE_COMMAND_MODE=$*" >&2
	@exit 2

# autoware command service use ROS_DOMAIN_ID from .env
autoware-request-initialpose:
	CMD="ros2 service call /set_initial_pose std_srvs/srv/Trigger '{}'" docker compose run --rm --no-deps autoware-command

autoware-request-control:
	CMD="ros2 topic pub -1 /awsim/control_mode_request_topic std_msgs/msg/Bool '{data: true}'" docker compose run --rm --no-deps autoware-command

# awsim admin service use ROS_DOMAIN_ID 0
awsim-request-start:
	@$(MAKE) awsim-request-start-$(AUTOWARE_COMMAND_MODE) \
		AWSIM_READY_DOMAINS="$(AWSIM_READY_DOMAINS)" \
		AUTOWARE_COMMAND_SERVICE="$(AUTOWARE_COMMAND_SERVICE)"

awsim-request-start-run:
	CMD="env ROS_DOMAIN_ID=0 AWSIM_READY_DOMAINS=$(AWSIM_READY_DOMAINS) bash /aichallenge/request_awsim_start.bash" \
		docker compose run --rm --no-deps "$(AUTOWARE_COMMAND_SERVICE)"

awsim-request-start-exec:
	docker compose exec -T -e AWSIM_READY_DOMAINS="$(AWSIM_READY_DOMAINS)" \
		"$(AUTOWARE_COMMAND_SERVICE)" env ROS_DOMAIN_ID=0 bash /aichallenge/request_awsim_start.bash

awsim-request-start-%:
	@echo "invalid AUTOWARE_COMMAND_MODE=$*" >&2
	@exit 2

awsim-request-start-and-watch-d1:
	@REPO_ROOT="$(CURDIR)" \
	RUN_ID="$(RUN_ID)" \
	LOG_DIR="$(LOG_DIR)" \
	RUN_HOST_DIR="$(RUN_HOST_DIR)" \
	AWSIM_READY_DOMAINS="$(AWSIM_READY_DOMAINS)" \
	D1_STALL_TIMEOUT_SEC="$(D1_STALL_TIMEOUT_SEC)" \
	D1_STALL_ENTER_SPEED_MPS="$(D1_STALL_ENTER_SPEED_MPS)" \
	D1_STALL_EXIT_SPEED_MPS="$(D1_STALL_EXIT_SPEED_MPS)" \
	D1_VELOCITY_FRESHNESS_SEC="$(D1_VELOCITY_FRESHNESS_SEC)" \
	D1_VELOCITY_EVIDENCE_FAILURE_SEC="$(D1_VELOCITY_EVIDENCE_FAILURE_SEC)" \
	AUTOWARE_COMMAND_SERVICE="$(AUTOWARE_COMMAND_SERVICE)" \
	AUTOWARE_COMMAND_MODE="$(AUTOWARE_COMMAND_MODE)" \
	AUTOWARE_RUNTIME_IMAGE="$(AUTOWARE_RUNTIME_IMAGE)" \
	bash aichallenge/run_awsim_with_d1_watchdog.bash

awsim-request-reset:
	CMD="env ROS_DOMAIN_ID=0 ros2 topic pub -1 /admin/awsim/reset std_msgs/msg/Empty '{}'" docker compose run --rm --no-deps autoware-command

# run simulator (docker compose up -d simulator)
simulator: capture-run-fingerprint
	@echo "Start AWSIM (SIM_MODE=$(SIM_MODE))"
	LOG_DIR="$(LOG_DIR)" SIM_MODE="$(SIM_MODE)" \
	AWSIM_START_MODE="$(AWSIM_START_MODE)" \
	AWSIM_START_COUNT_SECONDS="$(AWSIM_START_COUNT_SECONDS)" \
	AWSIM_VEHICLES="$(AWSIM_VEHICLES)" AWSIM_LAPS="$(AWSIM_LAPS)" \
	AWSIM_TIMEOUT="$(AWSIM_TIMEOUT)" AWSIM_EXTRA_ARGS="$(AWSIM_EXTRA_ARGS)" \
	ROS_DOMAIN_ID=0 docker compose up -d simulator

# racing kart (docker compose up -d driver)
driver:
	docker compose up -d driver

# zenoh (docker compose up -d zenoh)
zenoh:
	docker compose up -d zenoh

dev: SIM_MODE := dev
dev: RUN_KIND := dev
dev: AWSIM_START_MODE := sync
dev: AWSIM_VEHICLES := 1
dev: AWSIM_LAPS := 600
dev: AWSIM_TIMEOUT := 60000000
dev: simulator autoware-simulator
	@echo "Start dev simulation (AWSIM + Autoware)"
	@if [ "$(DEV_AUTO_START)" = "true" ]; then \
		status=0; \
		AWSIM_START_MODE="$(DEV_AUTO_START_EFFECTIVE_MODE)" AWSIM_READY_DOMAINS=1 \
		$(MAKE) awsim-request-start \
			RUN_ID="$(RUN_ID)" RUN_KIND="$@" \
			OUTPUT_ROOT="$(OUTPUT_ROOT)" OUTPUT_HOST_ROOT="$(OUTPUT_HOST_ROOT)" || status=$$?; \
		if [ "$$status" -ne 0 ]; then \
			echo "Automatic AWSIM Start failed; stop single-dev compose scope" >&2; \
			docker compose down --remove-orphans; \
			exit "$$status"; \
		fi; \
	elif [ "$(DEV_AUTO_START)" != "false" ]; then \
		echo "DEV_AUTO_START must be true or false: $(DEV_AUTO_START)" >&2; \
		exit 2; \
	fi
	@echo "To stop: make down  (docker compose down --remove-orphans)"

dev2: SIM_MODE := 2p
dev3: SIM_MODE := 3p
dev4: SIM_MODE := 4p
dev2 dev3 dev4: AWSIM_START_MODE := sync
dev2: AWSIM_VEHICLES := 2
dev3: AWSIM_VEHICLES := 3
dev4: AWSIM_VEHICLES := 4
dev2 dev3 dev4: AWSIM_LAPS := 6
dev2 dev3 dev4: AWSIM_TIMEOUT := 600
dev2: RUN_KIND := dev2
dev3: RUN_KIND := dev3
dev4: RUN_KIND := dev4
dev2: AWSIM_READY_DOMAINS := 1,2
dev3: AWSIM_READY_DOMAINS := 1,2,3
dev4: AWSIM_READY_DOMAINS := 1,2,3,4
dev2 dev4: AWSIM_START_TARGET := awsim-request-start
dev3: AWSIM_START_TARGET := awsim-request-start-and-watch-d1
dev2 dev3 dev4: simulator
	@N=$(@:dev%=%); \
	echo "Start $$N-vehicle dev (autoware on ROS_DOMAIN_ID 1..$$N via docker compose -p)"; \
	for p in $$(seq 1 $$N); do LOG_DIR=$(LOG_DIR) ROS_DOMAIN_ID=$$p docker compose -p $$p up -d autoware; done
	@AWSIM_READY_DOMAINS="$(AWSIM_READY_DOMAINS)" \
	$(MAKE) $(AWSIM_START_TARGET) \
		RUN_ID="$(RUN_ID)" RUN_KIND="$@" \
		OUTPUT_ROOT="$(OUTPUT_ROOT)" OUTPUT_HOST_ROOT="$(OUTPUT_HOST_ROOT)"
	@if [ "$(AWSIM_START_TARGET)" = "awsim-request-start" ]; then echo "To Stop: make down"; fi

gate1: GATE_SCENARIO := SafetyGate/scenario1.yaml
gate1: GATE_VEHICLES := 4
gate2: GATE_SCENARIO := SafetyGate/scenario2.yaml
gate2: GATE_VEHICLES := 4
gate3: GATE_SCENARIO := SafetyGate/scenario3.yaml
gate3: GATE_VEHICLES := 1
gate1 gate2 gate3: GATE_SCENARIO_ROOT := /aichallenge/simulator/AWSIM/AWSIM_Data/StreamingAssets

planner-pp-control-smoke: CONTROL_METHOD := state_lattice_pure_pursuit
planner-pp-control-smoke: RUN_KIND := planner-pp-control-smoke
planner-pp-control-smoke: PLANNER_PP_CONTROL_SMOKE_LIVE_SPATIAL := true
planner-pp-control-smoke: STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED := true
planner-pp-control-smoke: STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED := true
planner-pp-control-smoke: STATE_LATTICE_V4_POC_COMMAND_ACTIVATION_ENABLED := true
planner-pp-control-smoke: STATE_LATTICE_V2_PRODUCER_INSTANCE_ID := 4101
planner-pp-control-smoke: STATE_LATTICE_V2_PP_PRODUCER_INSTANCE_ID := 4201
planner-pp-control-smoke: STATE_LATTICE_V2_SESSION_ID := 1
planner-pp-control-smoke: gate2-live-authority-flags gate2-wrapper-authorization gate2-current-artifact-admission
	@if [ "$(CONTROL_METHOD)" != "state_lattice_pure_pursuit" ]; then \
		echo "planner-pp-control-smoke requires CONTROL_METHOD=state_lattice_pure_pursuit" >&2; \
		exit 2; \
	fi
	@$(MAKE) gate2 RUN_ID="$(RUN_ID)" \
		RUN_KIND="$(RUN_KIND)" \
		PLANNER_PP_CONTROL_SMOKE_LIVE_SPATIAL="$(PLANNER_PP_CONTROL_SMOKE_LIVE_SPATIAL)" \
		STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED="$(STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED)" \
		STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED="$(STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED)" \
		STATE_LATTICE_V2_PRODUCER_INSTANCE_ID="$(STATE_LATTICE_V2_PRODUCER_INSTANCE_ID)" \
		STATE_LATTICE_V2_PP_PRODUCER_INSTANCE_ID="$(STATE_LATTICE_V2_PP_PRODUCER_INSTANCE_ID)" \
		STATE_LATTICE_V2_SESSION_ID="$(STATE_LATTICE_V2_SESSION_ID)" \
		STATE_LATTICE_V4_POC_COMMAND_ACTIVATION_ENABLED="$(STATE_LATTICE_V4_POC_COMMAND_ACTIVATION_ENABLED)" \
		CONTROL_METHOD="state_lattice_pure_pursuit"

gate2-live-authority-flags:
	@if [ "$(STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED)" != "true" ] || \
		[ "$(STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED)" != "true" ] || \
		[ "$(STATE_LATTICE_V4_POC_COMMAND_ACTIVATION_ENABLED)" != "true" ]; then \
		echo "planner-pp-control-smoke requires V2 publish, V2 accept, and V4 activation to be true" >&2; \
		exit 2; \
	fi

gate2-wrapper-authorization:
	@env -i \
		PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
		HOME="/tmp" \
		PYTHONPATH="$(CURDIR)/tools/aic_test" \
		RUN_ID="$(RUN_ID)" \
		AIC_TEST_GATE2_REVIEWED_EXECUTION_ID="$${AIC_TEST_GATE2_REVIEWED_EXECUTION_ID-}" \
		AIC_TEST_GATE2_REVIEW_ATTEMPT_ID="$${AIC_TEST_GATE2_REVIEW_ATTEMPT_ID-}" \
		AIC_TEST_GATE2_REVIEW_LAUNCH_SPEC_SHA256="$${AIC_TEST_GATE2_REVIEW_LAUNCH_SPEC_SHA256-}" \
		AIC_TEST_GATE2_CAPABILITY_PATH="$${AIC_TEST_GATE2_CAPABILITY_PATH-}" \
		python3 -m aic_test --repo-root "$(CURDIR)" \
		verify-gate2-wrapper-capability --run-id "$(RUN_ID)"

gate2-current-artifact-admission:
	@check_artifact() { \
		name="$$1"; source_path="$$2"; install_path="$$3"; expected_target="$$4"; expected_sha="$$5"; \
		if [ -z "$$expected_sha" ] || ! [[ "$$expected_sha" =~ ^[0-9A-Fa-f]{64}$$ ]]; then \
			echo "GATE2_REQUIRED_$${name}_SHA256 must be a SHA-256" >&2; exit 2; \
		fi; \
		if [ ! -L "$$install_path" ] || [ "$$(readlink "$$install_path")" != "$$expected_target" ]; then \
			echo "Gate 2 install link mismatch for $$name" >&2; exit 2; \
		fi; \
		actual_sha="$$(sha256sum "$$source_path" | awk '{print $$1}')"; \
		if [ "$$actual_sha" != "$${expected_sha,,}" ]; then \
			echo "Gate 2 reviewed artifact mismatch for $$name" >&2; exit 2; \
		fi; \
	}; \
	check_artifact MUX_NODE \
		"aichallenge/workspace/src/aichallenge_submit/hybrid_control_mux/hybrid_control_mux/hybrid_control_mux_node.py" \
		"aichallenge/workspace/install/hybrid_control_mux/lib/hybrid_control_mux/hybrid_control_mux_node.py" \
		"/aichallenge/workspace/src/aichallenge_submit/hybrid_control_mux/hybrid_control_mux/hybrid_control_mux_node.py" \
		"$(GATE2_REQUIRED_MUX_NODE_SHA256)"; \
	check_artifact MUX_CONFIG \
		"aichallenge/workspace/src/aichallenge_submit/hybrid_control_mux/config/hybrid_control_mux.param.yaml" \
		"aichallenge/workspace/install/hybrid_control_mux/share/hybrid_control_mux/config/hybrid_control_mux.param.yaml" \
		"/aichallenge/workspace/src/aichallenge_submit/hybrid_control_mux/config/hybrid_control_mux.param.yaml" \
		"$(GATE2_REQUIRED_MUX_CONFIG_SHA256)"; \
	check_artifact PLANNER_NODE \
		"aichallenge/workspace/build/state_lattice_overtake_planner/state_lattice_overtake_planner_node" \
		"aichallenge/workspace/install/state_lattice_overtake_planner/lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node" \
		"/aichallenge/workspace/build/state_lattice_overtake_planner/state_lattice_overtake_planner_node" \
		"$(GATE2_REQUIRED_PLANNER_NODE_SHA256)"

planner-pp-control-smoke-eval: AUTOWARE_SERVICE := autoware-eval-runtime
planner-pp-control-smoke-eval: AUTOWARE_COMMAND_SERVICE := autoware-eval-command
planner-pp-control-smoke-eval: AUTOWARE_COMMAND_MODE := exec
planner-pp-control-smoke-eval: AUTOWARE_RUNTIME_IMAGE := aichallenge-2025-eval
planner-pp-control-smoke-eval: AUTOWARE_RUN_MODE := awsim-no-viz
planner-pp-control-smoke-eval: AUTOSTART_DEBUG_VISUALIZATION := false
planner-pp-control-smoke-eval: gate2-wrapper-authorization
	@$(MAKE) planner-pp-control-smoke \
		RUN_ID="$(RUN_ID)" AUTOWARE_SERVICE="$(AUTOWARE_SERVICE)" \
		AUTOWARE_COMMAND_SERVICE="$(AUTOWARE_COMMAND_SERVICE)" \
		AUTOWARE_COMMAND_MODE="$(AUTOWARE_COMMAND_MODE)" \
		AUTOWARE_RUNTIME_IMAGE="$(AUTOWARE_RUNTIME_IMAGE)" \
		GATE_EXTRA_ARGS="$(GATE_EXTRA_ARGS)" AWSIM_EXTRA_ARGS="$(AWSIM_EXTRA_ARGS)" \
		AUTOWARE_RUN_MODE="$(AUTOWARE_RUN_MODE)" \
		AUTOSTART_DEBUG_VISUALIZATION="$(AUTOSTART_DEBUG_VISUALIZATION)" \
		OUTPUT_HOST_ROOT="$(OUTPUT_HOST_ROOT)" OUTPUT_ROOT="$(OUTPUT_ROOT)"
gate1 gate3: AWSIM_START_TARGET := awsim-request-start
gate2: AWSIM_START_TARGET := awsim-request-start-and-watch-d1
gate2: AUTOWARE_RUN_MODE := awsim-no-viz
gate2: AUTOSTART_DEBUG_VISUALIZATION := false
gate2: gate2-wrapper-authorization
gate1 gate2 gate3:
	@echo "Start safety gate $(@:gate=%) ($(GATE_SCENARIO))"
	@base_args="--scenario $(GATE_SCENARIO_ROOT)/$(GATE_SCENARIO)"; \
	extra_args="$${AWSIM_EXTRA_ARGS:-} $(GATE_EXTRA_ARGS)"; \
	control_method="$${CONTROL_METHOD:-$(CONTROL_METHOD)}"; \
	start_mode="$${AWSIM_START_MODE:-sync}"; \
	$(MAKE) dev \
		RUN_ID="$(RUN_ID)" RUN_KIND="$(RUN_KIND)" \
		DEV_AUTO_START=false \
		RUN_GATE_SCENARIO="$(GATE_SCENARIO)" \
		AUTOWARE_RUN_MODE="$(AUTOWARE_RUN_MODE)" \
		AUTOSTART_DEBUG_VISUALIZATION="$(AUTOSTART_DEBUG_VISUALIZATION)" \
		ROSBAG=true CONTROL_METHOD="$$control_method" \
		AWSIM_START_MODE="$$start_mode" RACE_ARM_ON_VEHICLE_STATE="Start" \
		AWSIM_VEHICLES=$(GATE_VEHICLES) AWSIM_LAPS=unlimited \
		AWSIM_EXTRA_ARGS="$$base_args $$extra_args"
	@AWSIM_READY_DOMAINS=1 \
	$(MAKE) $(AWSIM_START_TARGET) \
		RUN_ID="$(RUN_ID)" RUN_KIND="$(RUN_KIND)" \
		OUTPUT_ROOT="$(OUTPUT_ROOT)" OUTPUT_HOST_ROOT="$(OUTPUT_HOST_ROOT)"

# Kept for backward compatibility; `make down` already cleans all projects.
down2 down3 down4: down

eval:
	@echo "Start evaluation simulation (AWSIM + Autoware)"
	docker compose up -d autoware-simulator-evaluation
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
	@if [ "$(AIC_TEST_SCOPED_PROJECT)" = "true" ]; then \
		docker compose down --remove-orphans; \
	else \
		for p in 1 2 3 4; do docker compose -p $$p down --remove-orphans; done; \
		docker compose down --remove-orphans; \
	fi

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
