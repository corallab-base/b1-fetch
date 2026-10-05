#!/bin/bash
# Put this repo's current commit on the other NX boards and build it there,
# so every board runs the same code from the same git history.
#
#   GitHub (origin) <-> .24 (development, internet) --push over SSH--> .23, .25
#
# .23/.25 have no internet, so only .24 talks to GitHub (git pull/push as
# usual); this script then pushes HEAD to ~/projects/fetch on the other
# boards (receive.denyCurrentBranch=updateInstead updates their checkout) and
# runs catkin_make there. It works from any board: by default it deploys to
# every board but the one it runs on.
#
# Usage:  ./deploy.sh            # all other boards
#         ./deploy.sh 23         # one board
# Commit first: it deploys HEAD, not uncommitted changes. Building is heavy:
# it refuses while a camera/ROS stack runs on the target board. A board with
# local edits refuses the push (fix or commit them from .24 instead).
set -e
cd "$(dirname "$0")"
ME=$(hostname -I | tr ' ' '\n' | grep -m1 '^192\.168\.123\.' | cut -d. -f4)
BOARDS=${@:-$(for b in 23 24 25; do [ "$b" = "$ME" ] || echo $b; done)}
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
  # realsense-ros is in the repo and built on /opt/ros. Except where OpenCV
  # 3.2's core library isn't at its system path (.24: moved to
  # ~/opencv3_backup_safe, so ROS's cv_bridge can't be linked against): there
  # the repo's realsense2_camera is skipped and ~/realsense_ws (built before
  # the move) is chained in instead.
  if ssh $host 'cd ~/projects/fetch && source /opt/ros/melodic/setup.bash &&
                if [ -e /usr/lib/aarch64-linux-gnu/libopencv_core.so.3.2.0 ]; then skip=""
                else skip=realsense2_camera; source ~/realsense_ws/devel/setup.bash
                     echo "no system OpenCV 3.2: using ~/realsense_ws for realsense2_camera"; fi &&
                nice catkin_make -j2 --force-cmake -DCATKIN_BLACKLIST_PACKAGES="$skip" >build.log 2>&1'; then
    echo "  build OK"
  else
    echo "  BUILD FAILED, last lines:"; ssh $host 'tail -15 ~/projects/fetch/build.log'; exit 1
  fi
done
