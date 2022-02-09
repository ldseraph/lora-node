CPU_N ?= $(shell nproc --all)
V = 0
Q = $(if $(filter 1,$V),,@)

.PHONY: clean build all

all: build

%_app:
	$Q scons --useconfig=./app/$*/.config  

menuconfig:
	$Q scons --menuconfig

build:
	$Q scons -j $(CPU_N)

clean:
	$Q scons --clean
