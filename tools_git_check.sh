#!/usr/bin/env bash
# Kill hung setup without matching this script name badly
pgrep -af tools_setup_ecole || true
pkill -f 'tools_setup_ecole.sh' 2>/dev/null || true
sleep 1

echo "==== remotes / status ===="
for d in environment limo teleoperation recording room207-limo group-informations; do
  p="$HOME/ros2_ws/src/$d"
  echo "-- $d --"
  if [ -d "$p/.git" ]; then
    git -C "$p" remote -v | head -1
    git -C "$p" status -sb
    git -C "$p" rev-parse --short HEAD
  else
    echo MISSING
  fi
done

echo "==== auth test (no prompt) ===="
export GIT_TERMINAL_PROMPT=0
export GIT_ASKPASS=echo
git -C "$HOME/ros2_ws/src/environment" ls-remote origin HEAD 2>&1 | tail -10 || true

echo "==== credential helper ===="
git config --global --get credential.helper || echo no_helper
ls -la ~/.git-credentials 2>/dev/null || echo no_credentials_file
