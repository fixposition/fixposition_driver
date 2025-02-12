#!/bin/bash
set -eEu
set -o pipefail
set -o errtrace

SCRIPTDIR=$(dirname $(readlink -f $0))
cd ${SCRIPTDIR}
pre-commit run --all-files --hook-stage manual
