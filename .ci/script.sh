#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

export PACKAGE_NAMES="$(./scripts/internal-get-packages.py distribution.yml ${REPOSITORY})"

./scripts/internal-build.sh ${PACKAGE_NAMES}
./scripts/internal-test.sh ${PACKAGE_NAMES}
# Check code style
./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --make-args check-format -- ada_demos
