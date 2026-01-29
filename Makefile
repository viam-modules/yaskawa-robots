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
	docker-amd64-ci

default: module.tar.gz

# format the source code
format:
	ls src/viam/*/*.*pp | xargs clang-format-19 -i --style=file

format-check:
	ls src/viam/*/*.*pp | xargs clang-format-19 -i --style=file --dry-run --Werror

configure:
	cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo -G Ninja

build: configure
	ninja -C build -j8


install: build
	ninja -C build install

test: build
	cmake --build build --target test

module.tar.gz: format-check install
	ninja -C build package

run-clang-tidy:
	clang-tidy-19 \
        -p build \
        --config-file ./.clang-tidy \
        --header-filter=".*/viam/*/*.*" \
	./src/viam/*/*.cpp

run-clang-check:
	clang-check-19 -p build ./src/viam/*/*.cpp 

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
