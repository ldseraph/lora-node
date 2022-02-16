CPU_N ?= $(shell nproc --all)
V = 0
Q = $(if $(filter 1,$V),,@)
APPS_NAME := $(addsuffix _app,$(filter-out app,$(notdir $(shell find ./app -maxdepth 1 -type d)))) 
VERSION ?= $(shell git describe --tags --dirty --match=v* 2> /dev/null || echo v0)

.PHONY: all
all: build

%_app:
	$Q scons --useconfig=./app/$*/.config

.PHONY: menuconfig
menuconfig:
	$Q scons --menuconfig

.PHONY: genconfig
genconfig:
	$Q scons --genconfig
	$Q if [ $(shell cat .config | grep -Po -c '(?<=APP_COMPONENTS_)\S+(?=\=y)') = 1 ]; then \
	cp .config ./app/$(shell cat .config | grep -Po '(?<=APP_COMPONENTS_)\S+(?=\=y)' | tr 'A-Z' 'a-z')/.config ; \
	fi

.PHONY: build
build:
	$Q scons -j $(CPU_N)

.PHONY: build_all
build_all:
	$Q for app in $(APPS_NAME) ; do \
		make $$app;\
		make build;\
	done

.PHONY: clean
clean:
	$Q scons --clean
	$Q rm *.bin 2> /dev/null || true
	$Q rm *.elf 2> /dev/null || true
	$Q rm *.map 2> /dev/null || true
	$Q rm *.objdump 2> /dev/null || true

.PHONY: version
version:
	@echo $(VERSION)
