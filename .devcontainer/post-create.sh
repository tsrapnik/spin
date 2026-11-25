#!/usr/bin/env bash
# Initialise / refresh the T2 west workspace in /workdir. Idempotent.
set -euo pipefail

sudo chown "$(id -u):$(id -g)" /workdir
cd /workdir

if [ ! -d .west ]; then
    west init -l spin
fi

west update --narrow --fetch-opt=--depth=1
west zephyr-export
