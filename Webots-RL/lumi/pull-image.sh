#!/bin/bash
set -euo pipefail
# Run on LUMI. The official image provides Python, Webots and Xvfb.
mkdir -p /projappl/project_462001326/webots-rl
cd /projappl/project_462001326/webots-rl
if [[ -e webots-R2025a.sif ]]; then
    echo 'Image already exists; refusing to overwrite it.' >&2
    exit 1
fi
singularity pull webots-R2025a.sif docker://cyberbotics/webots:R2025a-ubuntu22.04
sha256sum webots-R2025a.sif > webots-R2025a.sif.sha256
singularity exec --cleanenv webots-R2025a.sif bash -c 'command -v python3; command -v xvfb-run; command -v xauth; test -x /usr/local/webots/webots'
