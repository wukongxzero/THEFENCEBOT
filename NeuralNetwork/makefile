# --- Configuration ---
REMOTE_HOST = duckpcc@X.X.X.X
REMOTE_PORT = 8888
LOCAL_PORT  = 8888
MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(dir $(MKFILE_PATH))

SHELL := /bin/bash

# --- Commands ---

## help: Show this help message
help:
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@grep -E '^## [-a-zA-Z0-9_]+:' Makefile | sed 's/## //;s/:/	- /'

## launch: Start Jupyter Lab on the remote server (detached)
launch:
	source $(MKFILE_DIR).venv/bin/activate && jupyter lab --no-browser --port=$(REMOTE_PORT) && jupyter server list &

## tunnel: Create the SSH tunnel to forward the port
tunnel:
	@echo "Opening tunnel: http://localhost:$(LOCAL_PORT)"
	ssh -N -L $(LOCAL_PORT):localhost:$(REMOTE_PORT) $(REMOTE_HOST);
## start: Do both! Launch remote and open tunnel (requires two terminals or backgrounding)
start-remote:
	@echo "Starting Jupyter and Tunneling..."
	ssh $(REMOTE_HOST) "jupyter lab --no-browser --port=$(REMOTE_PORT)" &
	sleep 5
	ssh -N -L $(LOCAL_PORT):localhost:$(REMOTE_PORT) $(REMOTE_HOST)

setup-virt-env:
	mkdir .venv && python3 -m venv ./.venv
	source $(MKFILE_DIR).venv/bin/activate 
	pip install torch
	pip install jupyterlab
	pip install numpy
	


.PHONY: help launch tunnel start
