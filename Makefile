CPU_N ?= $(shell nproc --all)
V = 0
Q = $(if $(filter 1,$V),,@)

.PHONY: clean build all

all: build

%_app:
	$Q scons --useconfig=./app/$*/.config

menuconfig:
	$Q scons --menuconfig

genconfig:
	$Q scons --genconfig
	$Q if [ $(shell cat .config | grep -Po -c '(?<=APP_COMPONENTS_)\S+(?=\=y)') = 1 ]; then \
	cp .config ./app/$(shell cat .config | grep -Po '(?<=APP_COMPONENTS_)\S+(?=\=y)' | tr 'A-Z' 'a-z')/.config ; \
	fi

build:
	$Q scons -j $(CPU_N)

clean:
	$Q scons --clean
