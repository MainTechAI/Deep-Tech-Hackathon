#!/usr/bin/env bash
# Copy vision_nav to the server and (re)start it.
#   ./deploy/deploy.sh [ssh-host]      default ssh host: vpn
# First run creates server/.env on the server with random ROBOT_TOKEN / CONTROL_KEY.
set -euo pipefail
HOST="${1:-vpn}"
DIR=/opt/vision_nav
cd "$(dirname "$0")/.."

rsync -az --delete --exclude .venv --exclude __pycache__ --exclude 'server/.env' \
  --exclude 'firmware' ./ "$HOST:$DIR/"

ssh "$HOST" bash -s <<EOF
set -euo pipefail
cd $DIR
if [ ! -f server/.env ]; then
  cat > server/.env <<ENV
HTTP_HOST=127.0.0.1
ROBOT_TOKEN=\$(openssl rand -hex 12)
CONTROL_KEY=\$(openssl rand -hex 6)
ENV
  echo "created server/.env"
fi
cd deploy
docker compose up -d --build
docker compose ps
EOF
