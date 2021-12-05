THIS_DIR := $(dir $(abspath $(firstword $(MAKEFILE_LIST))))

install:
	CARGO_HOME=$(THIS_DIR)/../.cargo ROSRUST_MSG_PATH=/usr/share/ cargo install --path=$(srcdir) --root=$(DESTDIR) --no-track
