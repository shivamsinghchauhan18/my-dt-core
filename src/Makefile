


test:
	@echo "$(sep)Testing"
	@echo
	@echo "These commands run the unit tests."
	@echo
	@echo '- `make test-all`:              Run all the tests.'
	@echo
	@echo '- `make test-circle`:           The tests to run in continuous integration. .'
	@echo '- `make test-catkin_tests`:     Run the ROS tests.'
	@echo '- `make test-anti_instagram`:   Run the `anti_instagram` tests.'
	@echo '- `make test-comptests`:        Run the `comptests` tests.'
	@echo '- `make test-comptests-clean`:        Run the `comptests` tests.'
	@echo '- `make test-comptests-collect-junit`: Collects the JUnit results.'
	@echo '- `make test-download-logs`: Downloads the logs needed for the tests.'
	@echo
	@echo

check-environment:

test-circle: \
	test-comptests-circle \
	test-download-logs \
	test-misc-utils


	#test-line-detector-programmatic

#
# test-catkin_tests \
# test-anti_instagram
#

test-all: \
	test-comptests \
	test-download-logs \
	test-catkin_tests \
	test-misc-utils

### Comptests

comptests_packages=\
	easy_node_tests\
	easy_logs_tests\
	easy_algo_tests\
	line_detector2_tests\
	complete_image_pipeline_tests\
	duckietown_segmaps_tests\
	lane_filter_generic_tests\
	easy_regression_tests\
	grid_helper_tests

# These take a long time
# anti_instagram_tests\

comptests_out=out/comptests

test-comptests-clean:
	-rm -rf $(comptests_out)

test-comptests-again:
	$(MAKE) test-comptests-clean
	$(MAKE) test-comptests

test-comptests:  test-download-logs
	comptests -o $(comptests_out) --nonose --contracts -c "rparmake" $(comptests_packages)

test-comptests-circle:  test-download-logs
	# comptests -o $(comptests_out) --nonose --contracts -c "rparmake n=3" $(comptests_packages)
	comptests --circle -o $(comptests_out) --nonose -c "rparmake n=4" $(comptests_packages)

test-comptests-slow:  test-download-logs
	comptests -o $(comptests_out) --nonose --contracts -c "rmake" $(comptests_packages)

test-comptests-collect-junit:
	mkdir -p $(comptests_out)/junit
	comptests-to-junit $(comptests_out)/compmake > $(comptests_out)/junit/junit.xml

test-catkin_tests: check-environment
	catkin --workspace=$(CATKIN_WS_DIR)  run_tests
	#bash -c "source environment.sh; catkin_make -C $(catkin_ws) run_tests; catkin_test_results $(catkin_ws)/build/test_results/"

# onelog=20160223-amadoa-amadobot-RCDP2
onelog=2016-04-29-dp3auto-neptunus-1

test-download-logs:
	@echo Loading log
	rosrun easy_logs download $(onelog) tori_ETHZ_2017-12-22-17-18-41

test-misc-utils:
	rosrun complete_image_pipeline validate_calibration robbie
	rosrun complete_image_pipeline display_segmaps 'DT17*tile*'

test-cloud-logs: cloud-download
	rosrun easy_logs summary --cloud  $(onelog)





# Docker build configuration
REPO_NAME=dt-core
DISTRO=daffy
ARCH=amd64
DOCKER_REGISTRY=docker.io
BASE_TAG=${DISTRO}-${ARCH}

# Multi-architecture support
SUPPORTED_ARCHS=amd64,arm64
BUILDX_BUILDER=dt-core-builder
BUILDX_PLATFORM=linux/amd64,linux/arm64

# Build targets
BUILD_TARGETS=development testing production
DEFAULT_TARGET=production

# Image tags
tag_dev=duckietown/$(REPO_NAME):$(DISTRO)-devel-$(ARCH)
tag_test=duckietown/$(REPO_NAME):$(DISTRO)-test-$(ARCH)
tag_prod=duckietown/$(REPO_NAME):$(DISTRO)-$(ARCH)

# Multi-arch image tags
tag_dev_multi=duckietown/$(REPO_NAME):$(DISTRO)-devel-multi
tag_test_multi=duckietown/$(REPO_NAME):$(DISTRO)-test-multi
tag_prod_multi=duckietown/$(REPO_NAME):$(DISTRO)-multi

# Enhanced Docker build system
build-help:
	@echo "$(sep)Enhanced Docker Build System"
	@echo
	@echo "Multi-stage build targets:"
	@echo "- make build-dev:        Build development image with debugging tools"
	@echo "- make build-test:       Build testing image with test frameworks"
	@echo "- make build-prod:       Build production image (optimized)"
	@echo "- make build-all:        Build all targets"
	@echo
	@echo "Multi-architecture builds:"
	@echo "- make buildx-setup:     Setup Docker buildx for multi-arch builds"
	@echo "- make buildx-dev:       Build development image for multiple architectures"
	@echo "- make buildx-test:      Build testing image for multiple architectures"
	@echo "- make buildx-prod:      Build production image for multiple architectures"
	@echo "- make buildx-all:       Build all targets for multiple architectures"
	@echo "- make buildx-info:      Show buildx builder information"
	@echo "- make buildx-clean:     Clean buildx builder and cache"
	@echo
	@echo "Development commands:"
	@echo "- make shell-dev:        Interactive shell in development container"
	@echo "- make shell-test:       Interactive shell in testing container"
	@echo "- make shell-prod:       Interactive shell in production container"
	@echo "- make shell-mount-dev:  Development shell with source code mounted"
	@echo
	@echo "macOS development:"
	@echo "- make macos-setup:      Setup X11 forwarding for macOS"
	@echo "- make shell-macos:      Development shell with X11 forwarding"
	@echo
	@echo "Build monitoring:"
	@echo "- make build-info:       Show build information and logs"
	@echo "- make build-clean:      Clean build artifacts"
	@echo "- make build-validate:   Validate build system configuration"
	@echo

# Multi-stage builds
build-dev:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Building development image..."
	docker build \
		--target development \
		--build-arg BUILD_TARGET=development \
		--build-arg ARCH=$(ARCH) \
		--build-arg DISTRO=$(DISTRO) \
		-t $(tag_dev) \
		.
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Development image built: $(tag_dev)"

build-test:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Building testing image..."
	docker build \
		--target testing \
		--build-arg BUILD_TARGET=testing \
		--build-arg ARCH=$(ARCH) \
		--build-arg DISTRO=$(DISTRO) \
		-t $(tag_test) \
		.
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Testing image built: $(tag_test)"

build-prod:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Building production image..."
	docker build \
		--target production \
		--build-arg BUILD_TARGET=production \
		--build-arg ARCH=$(ARCH) \
		--build-arg DISTRO=$(DISTRO) \
		-t $(tag_prod) \
		.
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Production image built: $(tag_prod)"

build-all: build-dev build-test build-prod
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] All build targets completed"

# Interactive shells
shell-dev:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Starting development shell..."
	docker run -it --rm $(tag_dev) bash

shell-test:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Starting testing shell..."
	docker run -it --rm $(tag_test) bash

shell-prod:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Starting production shell..."
	docker run -it --rm $(tag_prod) bash

# Development with mounted source code
shell-mount-dev:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Starting development shell with mounted source..."
	docker run -it --rm \
		-v $(PWD)/Makefile:/code/catkin_ws/src/dt-core/Makefile \
		-v $(PWD)/packages:/code/catkin_ws/src/dt-core/packages \
		-v $(PWD)/out:/code/catkin_ws/src/dt-core/out \
		-v $(PWD)/docs:/code/catkin_ws/src/dt-core/docs \
		-v $(PWD)/scripts:/code/catkin_ws/src/dt-core/scripts \
		$(tag_dev) \
		bash -c "echo 'Development environment ready with mounted source code'; source /opt/ros/noetic/setup.bash; source /code/catkin_ws/devel/setup.bash; bash"

# macOS development support
macos-setup:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Setting up macOS X11 forwarding..."
	@echo "Please ensure XQuartz is installed and running:"
	@echo "1. Install XQuartz: brew install --cask xquartz"
	@echo "2. Start XQuartz and enable 'Allow connections from network clients'"
	@echo "3. Run: xhost +localhost"
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] macOS setup instructions displayed"

shell-macos:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Starting macOS development shell with X11 forwarding..."
	docker run -it --rm \
		-e DISPLAY=host.docker.internal:0 \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(PWD)/packages:/code/catkin_ws/src/dt-core/packages \
		-v $(PWD)/out:/code/catkin_ws/src/dt-core/out \
		$(tag_dev) \
		bash -c "echo 'macOS development environment ready'; echo 'X11 forwarding enabled for GUI applications'; source /opt/ros/noetic/setup.bash; source /code/catkin_ws/devel/setup.bash; bash"

# Build information and monitoring
build-info:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Build Information:"
	@echo "Repository: $(REPO_NAME)"
	@echo "Distribution: $(DISTRO)"
	@echo "Architecture: $(ARCH)"
	@echo "Available images:"
	@docker images | grep $(REPO_NAME) || echo "No images found"
	@echo
	@echo "Build logs from containers:"
	@docker run --rm $(tag_dev) cat /tmp/build_info.txt 2>/dev/null || echo "Development image not available"
	@docker run --rm $(tag_test) cat /tmp/build_info.txt 2>/dev/null || echo "Testing image not available"
	@docker run --rm $(tag_prod) cat /tmp/build_info.txt 2>/dev/null || echo "Production image not available"

build-clean:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Cleaning build artifacts..."
	docker system prune -f
	docker image prune -f
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Build cleanup completed"

# Multi-architecture build support with Docker buildx
buildx-setup:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Setting up Docker buildx for multi-architecture builds..."
	@docker buildx inspect $(BUILDX_BUILDER) >/dev/null 2>&1 || \
		docker buildx create --name $(BUILDX_BUILDER) --driver docker-container --bootstrap
	@docker buildx use $(BUILDX_BUILDER)
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Docker buildx builder '$(BUILDX_BUILDER)' ready"
	@echo "Supported platforms: $(BUILDX_PLATFORM)"

buildx-dev: buildx-setup
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Building multi-architecture development image..."
	docker buildx build \
		--platform $(BUILDX_PLATFORM) \
		--target development \
		--build-arg BUILD_TARGET=development \
		--build-arg DISTRO=$(DISTRO) \
		-t $(tag_dev_multi) \
		--push \
		.
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Multi-architecture development image built: $(tag_dev_multi)"

buildx-test: buildx-setup
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Building multi-architecture testing image..."
	docker buildx build \
		--platform $(BUILDX_PLATFORM) \
		--target testing \
		--build-arg BUILD_TARGET=testing \
		--build-arg DISTRO=$(DISTRO) \
		-t $(tag_test_multi) \
		--push \
		.
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Multi-architecture testing image built: $(tag_test_multi)"

buildx-prod: buildx-setup
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Building multi-architecture production image..."
	docker buildx build \
		--platform $(BUILDX_PLATFORM) \
		--target production \
		--build-arg BUILD_TARGET=production \
		--build-arg DISTRO=$(DISTRO) \
		-t $(tag_prod_multi) \
		--push \
		.
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Multi-architecture production image built: $(tag_prod_multi)"

buildx-all: buildx-dev buildx-test buildx-prod
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] All multi-architecture builds completed"

buildx-info:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Docker buildx information:"
	@echo "Active builder: $(BUILDX_BUILDER)"
	@echo "Supported platforms: $(BUILDX_PLATFORM)"
	@docker buildx inspect $(BUILDX_BUILDER) 2>/dev/null || echo "Builder not found - run 'make buildx-setup'"
	@echo
	@echo "Available multi-architecture images:"
	@docker buildx imagetools inspect $(tag_dev_multi) 2>/dev/null || echo "Development multi-arch image not found"
	@docker buildx imagetools inspect $(tag_test_multi) 2>/dev/null || echo "Testing multi-arch image not found"
	@docker buildx imagetools inspect $(tag_prod_multi) 2>/dev/null || echo "Production multi-arch image not found"

buildx-clean:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Cleaning buildx builder and cache..."
	@docker buildx rm $(BUILDX_BUILDER) 2>/dev/null || echo "Builder $(BUILDX_BUILDER) not found"
	@docker buildx prune -f
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Buildx cleanup completed"

# Build optimization and validation
build-validate:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Validating build system configuration..."
	@echo "Repository: $(REPO_NAME)"
	@echo "Distribution: $(DISTRO)"
	@echo "Default architecture: $(ARCH)"
	@echo "Supported architectures: $(SUPPORTED_ARCHS)"
	@echo "Docker registry: $(DOCKER_REGISTRY)"
	@echo
	@echo "Checking Docker buildx availability..."
	@docker buildx version >/dev/null 2>&1 && echo "✓ Docker buildx available" || echo "✗ Docker buildx not available"
	@echo
	@echo "Checking dependency files..."
	@test -f dependencies-py3.txt && echo "✓ dependencies-py3.txt found" || echo "✗ dependencies-py3.txt missing"
	@test -f dependencies-py3.dt.txt && echo "✓ dependencies-py3.dt.txt found" || echo "✗ dependencies-py3.dt.txt missing"
	@test -f dependencies-py3.mock.txt && echo "✓ dependencies-py3.mock.txt found" || echo "✗ dependencies-py3.mock.txt missing"
	@echo
	@echo "Checking advanced dependencies..."
	@grep -q "torch" dependencies-py3.txt && echo "✓ PyTorch dependencies found" || echo "✗ PyTorch dependencies missing"
	@grep -q "ultralytics" dependencies-py3.txt && echo "✓ YOLO dependencies found" || echo "✗ YOLO dependencies missing"
	@grep -q "scipy" dependencies-py3.txt && echo "✓ SciPy dependencies found" || echo "✗ SciPy dependencies missing"
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Build system validation completed"

# Architecture-specific builds
build-amd64:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Building for AMD64 architecture..."
	$(MAKE) ARCH=amd64 build-prod

build-arm64:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Building for ARM64 architecture..."
	$(MAKE) ARCH=arm64 build-prod

build-multi-arch: build-amd64 build-arm64
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Multi-architecture builds completed"

# Production optimization flags
PROD_BUILD_ARGS=--build-arg BUILDKIT_INLINE_CACHE=1 \
				--build-arg BUILD_OPTIMIZATION=true \
				--build-arg PYTHON_OPTIMIZE=2

build-prod-optimized:
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Building optimized production image..."
	docker build \
		--target production \
		--build-arg BUILD_TARGET=production \
		--build-arg ARCH=$(ARCH) \
		--build-arg DISTRO=$(DISTRO) \
		$(PROD_BUILD_ARGS) \
		-t $(tag_prod) \
		.
	@echo "$(date '+%Y-%m-%d %H:%M:%S') [MAKE] Optimized production image built: $(tag_prod)"

# Legacy compatibility
shell: shell-prod
shell-mount: shell-mount-dev
docker-test: build-test
