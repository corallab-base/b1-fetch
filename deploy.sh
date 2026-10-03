#!/bin/bash
# Put this repo's current commit on the other NX boards and build it there,
# so every board runs the same code from the same git history.
#
#   GitHub (origin) <-> .24 (this board) --push over SSH--> .23, .25
#
# .23/.25 have no internet, so only .24 talks to GitHub (git pull/push as
# usual); this script then pushes HEAD to ~/projects/fetch on the other
# boards (receive.denyCurrentBranch=updateInstead updates their checkout) and
# runs catkin_make there.
#
# Usage:  ./deploy.sh            # .23 and .25
#         ./deploy.sh 23         # one board
# Commit first: it deploys HEAD, not uncommitted changes. Building is heavy:
# it refuses while a camera/ROS stack runs on the target board. A board with
# local edits refuses the push (fix or commit them from .24 instead).
set -e
cd "$(dirname "$0")"
BOARDS=${@:-23 25}
[ -z "$(git status --porcelain --untracked-files=no)" ] || echo "note: uncommitted changes here are NOT deployed (deploying $(git rev-parse --short HEAD))"
BRANCH=$(git rev-parse --abbrev-ref HEAD)

for b in $BOARDS; do
  host=unitree@192.168.123.$b
  echo "== .$b"
  if ssh -o BatchMode=yes $host 'pgrep -f "[r]ealsense|[n]odelet|[r]oslaunch|[r]osmaster" >/dev/null'; then
    echo "  ROS/camera processes running on .$b: stop the bringup first. Skipped."; continue
  fi
  # one-time: turn the board's existing workspace into a checkout of this repo
  if ! ssh $host 'test -d ~/projects/fetch/.git'; then
    echo "  first deploy: making ~/projects/fetch a git checkout (old remote_env.sh kept as remote_env.sh.pre-git)"
    ssh $host "cd ~/projects/fetch && git init -q && git config receive.denyCurrentBranch updateInstead &&
               { [ ! -f remote_env.sh ] || mv remote_env.sh remote_env.sh.pre-git; }"
    git push -q ssh://$host/home/unitree/projects/fetch HEAD:refs/heads/incoming
    ssh $host "cd ~/projects/fetch && git checkout -q -f -B $BRANCH incoming && git branch -q -D incoming"
  else
    git push -q ssh://$host/home/unitree/projects/fetch HEAD:refs/heads/$BRANCH
  fi
  echo "  at $(ssh $host 'cd ~/projects/fetch && git log --oneline -1')"
  echo "  building (log: ~/projects/fetch/build.log on .$b)"
  if ssh $host 'cd ~/projects/fetch && source /opt/ros/melodic/setup.bash && nice catkin_make -j2 >build.log 2>&1'; then
    echo "  build OK"
  else
    echo "  BUILD FAILED, last lines:"; ssh $host 'tail -15 ~/projects/fetch/build.log'; exit 1
  fi
done
