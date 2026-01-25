# Dockerized cross build helper for libshieldcomm (Raspberry Pi)

IMAGE  ?= stm32build:latest
# aarch64 (RPi 64-bit) or armhf (RPi 32-bit) or native
TARGET ?= aarch64

# Container run helper
DOCKER_RUN = docker run --rm -t \
  -u $(shell id -u):$(shell id -g) \
  -v $(CURDIR)/..:/workspace \
  -w /workspace/libshieldcomm

# Toolchain selection
ifeq ($(TARGET),aarch64)
  CXX := aarch64-linux-gnu-g++
  CC  := aarch64-linux-gnu-gcc
  AR  := aarch64-linux-gnu-ar
  STRIP := aarch64-linux-gnu-strip
else ifeq ($(TARGET),armhf)
  CXX := arm-linux-gnueabihf-g++
  CC  := arm-linux-gnueabihf-gcc
  AR  := arm-linux-gnueabihf-ar
  STRIP := arm-linux-gnueabihf-strip
else ifeq ($(TARGET),native)
  CXX := g++
  CC  := gcc
  AR  := ar
  STRIP := strip
else
  $(error Unsupported TARGET=$(TARGET). Use aarch64|armhf|native)
endif

BUILD_DIR := build/$(TARGET)
CMAKE ?= cmake
CMAKE_BUILD_TYPE ?= Release
CMAKE_ARGS := -S . -B $(BUILD_DIR) \
  -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) \
  -DCMAKE_C_COMPILER=$(CC) \
  -DCMAKE_CXX_COMPILER=$(CXX)

DEB_NAME := libshieldcomm
DEB_DEV_NAME := libshieldcomm-dev
DEB_VERSION ?= 0.1.0
DEB_REVISION ?= 1
DEB_MAINTAINER ?= Slot Hat <dev@localhost>
DEB_SECTION ?= libs
DEB_PRIORITY ?= optional
DEB_DESCRIPTION ?= ShieldComm serial protocol library
DEB_VERSION_FULL = $(DEB_VERSION)-$(DEB_REVISION)

ifeq ($(TARGET),aarch64)
  DEB_ARCH = arm64
else ifeq ($(TARGET),armhf)
  DEB_ARCH = armhf
else ifeq ($(TARGET),native)
  DEB_ARCH = $(shell dpkg --print-architecture)
endif

DEB_MULTIARCH = $(shell $(CXX) -dumpmachine)
DEB_LIBDIR = /usr/lib/$(DEB_MULTIARCH)
DEB_OUT = $(BUILD_DIR)/deb
DEB_STAGE = $(DEB_OUT)/stage
DEB_LIB_ROOT = $(DEB_STAGE)/$(DEB_NAME)
DEB_DEV_ROOT = $(DEB_STAGE)/$(DEB_DEV_NAME)
DEB_LIB_PKG = $(DEB_OUT)/$(DEB_NAME)_$(DEB_VERSION_FULL)_$(DEB_ARCH).deb
DEB_DEV_PKG = $(DEB_OUT)/$(DEB_DEV_NAME)_$(DEB_VERSION_FULL)_$(DEB_ARCH).deb

.PHONY: all help docker-build build clean shell show-toolchain deb deb-internal deb-clean

all: build

help:
	@echo "Targets:"
	@echo "  docker-build  - Build docker image $(IMAGE)"
	@echo "  build         - Build library inside container (TARGET=$(TARGET))"
	@echo "  clean         - Remove build/$(TARGET)"
	@echo "  deb           - Build .deb packages for libshieldcomm"
	@echo "  deb-clean     - Remove deb output directory"
	@echo "  shell         - Open shell in container"
	@echo "  show-toolchain- Print selected compilers"
	@echo ""
	@echo "Variables:"
	@echo "  IMAGE=<name:tag> (default: $(IMAGE))"
	@echo "  TARGET=aarch64|armhf|native (default: $(TARGET))"
	@echo "  DEB_VERSION=<version> (default: $(DEB_VERSION))"
	@echo "  DEB_REVISION=<n>      (default: $(DEB_REVISION))"

docker-build:
	docker build -t $(IMAGE) ..

show-toolchain:
	@echo "TARGET=$(TARGET)"
	@echo "CXX=$(CXX)"
	@echo "CC=$(CC)"
	@echo "AR=$(AR)"

# Host-side target: runs container build
build:
	$(DOCKER_RUN) $(IMAGE) bash -lc 'make build-internal TARGET=$(TARGET)'

# Internal build executed inside container
build-internal:
	$(CMAKE) $(CMAKE_ARGS)
	$(CMAKE) --build $(BUILD_DIR) -- -j$(shell nproc)

deb:
	$(DOCKER_RUN) $(IMAGE) bash -lc 'make deb-internal TARGET=$(TARGET) DEB_VERSION=$(DEB_VERSION) DEB_REVISION=$(DEB_REVISION) DEB_MAINTAINER="$(DEB_MAINTAINER)"'

deb-internal: build-internal
	rm -rf $(DEB_STAGE)
	mkdir -p $(DEB_LIB_ROOT)/DEBIAN $(DEB_DEV_ROOT)/DEBIAN
	mkdir -p $(DEB_LIB_ROOT)$(DEB_LIBDIR) $(DEB_DEV_ROOT)$(DEB_LIBDIR)
	mkdir -p $(DEB_DEV_ROOT)/usr/include/shieldcomm
	cp -a $(BUILD_DIR)/libshieldcomm.so.* $(DEB_LIB_ROOT)$(DEB_LIBDIR)/
	cp -a $(BUILD_DIR)/libshieldcomm.so $(DEB_DEV_ROOT)$(DEB_LIBDIR)/
	cp -a $(BUILD_DIR)/libshieldcomm.a $(DEB_DEV_ROOT)$(DEB_LIBDIR)/
	cp -a include/shieldcomm/*.hpp $(DEB_DEV_ROOT)/usr/include/shieldcomm/
	printf "Package: $(DEB_NAME)\nVersion: $(DEB_VERSION_FULL)\nSection: $(DEB_SECTION)\nPriority: $(DEB_PRIORITY)\nArchitecture: $(DEB_ARCH)\nMaintainer: $(DEB_MAINTAINER)\nDescription: $(DEB_DESCRIPTION)\n" > $(DEB_LIB_ROOT)/DEBIAN/control
	printf "Package: $(DEB_DEV_NAME)\nVersion: $(DEB_VERSION_FULL)\nSection: $(DEB_SECTION)\nPriority: $(DEB_PRIORITY)\nArchitecture: $(DEB_ARCH)\nDepends: $(DEB_NAME) (= $(DEB_VERSION_FULL))\nMaintainer: $(DEB_MAINTAINER)\nDescription: $(DEB_DESCRIPTION) (development files)\n" > $(DEB_DEV_ROOT)/DEBIAN/control
	mkdir -p $(DEB_OUT)
	dpkg-deb --build --root-owner-group $(DEB_LIB_ROOT) $(DEB_LIB_PKG)
	dpkg-deb --build --root-owner-group $(DEB_DEV_ROOT) $(DEB_DEV_PKG)

clean:
	$(DOCKER_RUN) $(IMAGE) bash -lc 'rm -rf build/$(TARGET)'

deb-clean:
	$(DOCKER_RUN) $(IMAGE) bash -lc 'rm -rf $(DEB_OUT)'

shell:
	$(DOCKER_RUN) -it $(IMAGE) bash
