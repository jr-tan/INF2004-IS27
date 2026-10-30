# Host (PC) unit tests for subsystem 1's codec.  Needs only gcc and make.
#   make        build and run
CC      ?= gcc
SRC_DIR := ../../../app_program/subsystem1
CFLAGS  := -std=c11 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror -O1 -g

test_codec: test_codec.c $(SRC_DIR)/bitpack.c $(SRC_DIR)/comms_frames.c
	$(CC) $(CFLAGS) -o $@ $^

run: test_codec
	./test_codec

all: run
.PHONY: run all
clean:
	rm -f test_codec
