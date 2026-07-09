.PHONY: \
	module.tar.gz \
	format \
	format-check \
	test \
	run-clang-tidy \
	run-clang-check \
	clean \
	clean-all \
	docker-amd64 \
	docker-arm64 \
	docker-build \
	docker-upload \
	docker \
	docker-arm64-ci \
	docker-amd64-ci \
	get-firmware

default: module.tar.gz

# format the source code
format:
	ls src/viam/*/*.*pp | xargs clang-format-19 -i --style=file

format-check:
	ls src/viam/*/*.*pp | xargs clang-format-19 -i --style=file --dry-run --Werror

configure:
	cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo -G Ninja

build: configure
	ninja -C build yaskawa-robots


install: build
	ninja -C build install

test: configure
	cmake --build build --target broadcast_log_parser_test realtime_trajectory_logger_test move_limit_test fault_injection_test controller_integration_test
	ctest --test-dir build --output-on-failure

module.tar.gz: format-check install
	ninja -C build package

run-clang-tidy:
	run-clang-tidy-19 \
        -p build \
        -config-file ./.clang-tidy \
        -header-filter=".*/src/viam/(lib|module)/.*" \
        -extra-arg=-D_Bool=bool \
	./src/viam/*/*.cpp

run-clang-check:
	clang-check-19 -p build --extra-arg=-D_Bool=bool ./src/viam/*/*.cpp

# MotoPlus firmware is distributed out-of-band via GCS, not committed. The bucket holds
# immutable viammoto-<git-describe>.out objects. The shipped version is pinned in the committed
# ./firmware.version so releases are reproducible and the module flashes the firmware it was
# built against (bump that file to ship a new one). get-firmware reads the pin, downloads that
# object to a stable name so the runtime path never changes, and writes a sidecar .version
# (bundled into the tarball) with the id the flasher compares against. The .out lands under src/
# so conan export_sources (src/*) picks it up and the install() rule in CMakeLists.txt bundles it.
FIRMWARE_BUCKET = gs://yaskawa-firmware.viam.dev
FIRMWARE_DIR = src/firmware
FIRMWARE_OUT = $(FIRMWARE_DIR)/viammoto.out
FIRMWARE_VERSION_FILE = firmware.version

get-firmware:
	mkdir -p $(FIRMWARE_DIR)
	if [ ! -f $(FIRMWARE_VERSION_FILE) ]; then echo "get-firmware: $(FIRMWARE_VERSION_FILE) missing" >&2; exit 1; fi; \
	ver=$$(grep -v '^[[:space:]]*#' $(FIRMWARE_VERSION_FILE) | tr -d '[:space:]'); \
	if [ -z "$$ver" ]; then echo "get-firmware: $(FIRMWARE_VERSION_FILE) has no version" >&2; exit 1; fi; \
	case "$$ver" in *dirty*) echo "get-firmware: refusing dirty firmware '$$ver'" >&2; exit 1;; esac; \
	echo "get-firmware: fetching viammoto-$$ver.out"; \
	gsutil cp $(FIRMWARE_BUCKET)/viammoto-$$ver.out $(FIRMWARE_OUT); \
	printf '%s' "$$ver" > $(FIRMWARE_OUT).version

clean:
	rm -rf build

clean-all:
	git clean -fxd

# Docker
BUILD_CMD = docker buildx build --pull $(BUILD_PUSH) --force-rm --build-arg MAIN_TAG=$(MAIN_TAG) \
	--build-arg BASE_TAG=$(BUILD_TAG) --platform linux/$(BUILD_TAG) -f $(BUILD_FILE) -t '$(MAIN_TAG):$(BUILD_TAG)' .
BUILD_PUSH = --load
BUILD_FILE = Dockerfile

docker-amd64: MAIN_TAG = ghcr.io/viam-modules/yaskawa-robots
docker-amd64: BUILD_TAG = amd64
docker-amd64:
	$(BUILD_CMD)

docker-arm64: MAIN_TAG = ghcr.io/viam-modules/yaskawa-robots
docker-arm64: BUILD_TAG = arm64
docker-arm64:
	$(BUILD_CMD)

docker-build: docker-amd64 docker-arm64

docker-upload:
	docker push 'ghcr.io/viam-modules/yaskawa-robots:amd64'
	docker push 'ghcr.io/viam-modules/yaskawa-robots:arm64'

docker: docker-build docker-upload

# CI targets that automatically push, avoid for local test-first-then-push workflows
docker-arm64-ci: MAIN_TAG = ghcr.io/viam-modules/yaskawa-robots
docker-arm64-ci: BUILD_TAG = arm64
docker-arm64-ci: BUILD_PUSH = --push
docker-arm64-ci:
	$(BUILD_CMD)

docker-amd64-ci: MAIN_TAG = ghcr.io/viam-modules/yaskawa-robots
docker-amd64-ci: BUILD_TAG = amd64
docker-amd64-ci: BUILD_PUSH = --push
docker-amd64-ci:
	$(BUILD_CMD)
