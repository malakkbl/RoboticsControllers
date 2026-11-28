# Hardware deployment plan (PID on the real robot)

Status: the PID controller works in simulation. Nothing below has been run on a robot yet.
Items marked **TODO** or **TO CONFIRM** are unknown until we have the hardware or the SSH access.

Two possible routes:

- **Route A:** SSH into the professor's robot (already set up).
- **Route B:** our own Raspberry Pi 4 with a screen, running everything from there.

Both routes need the same code additions (see "What is missing in the folder" below).

---

## What the controller needs on the robot (both routes)

- Python 3 with **numpy** and **pyyaml** only. `matplotlib`, `pandas` and `jupyter` from `requirements.txt` are for the notebook and are not needed on the robot (the `common/` and `controllers/` packages do not import them).
- ROS 2 (for the node that is still to be written).
- A pose topic (`/robot_pose`) in the same coordinate frame as the path.
- A velocity topic (`/cmd_vel`, `Twist`) that the robot listens to.
- Run from the project root, because the imports are `from common...` / `from controllers...`.

Only these config values matter on the real robot: `controllers.pid.*`, `robot.v_max`, `robot.w_max`, `sim.goal_tolerance`.
`cmd_delay_steps`, `a_max`, `alpha_max`, `pos_std`, `yaw_std`, `w_bias` only configure the simulator.

---

## Route A: SSH into the professor's robot

It is someone else's robot: look first, keep our files in our own folder, test carefully.

### A0. Before connecting
- [ ] Get from the professor: SSH user, address, password/key.
- [ ] Ask him: how is the robot software launched, how do we stop it in an emergency, is the joystick override active (the notebook mentions the joystick at priority 100, above our controller at 50).
- [ ] Do not commit the SSH details to the repo or this file.

### A1. Connect and look around (read-only)
```bash
ssh <user>@<address>
whoami && hostname
ls ~
echo $ROS_DISTRO                 # empty? try: source /opt/ros/*/setup.bash
ros2 node list                   # is the robot software already running?
ros2 topic list
ros2 topic info /robot_pose -v   # message type, who publishes
ros2 topic info /cmd_vel -v      # message type, who listens
ros2 topic hz /robot_pose        # pose rate (Ctrl+C to stop)
ros2 topic echo /robot_pose --once
python3 --version
python3 -c "import numpy, yaml; print('ok')"
```
Write the results in the "Results" section at the bottom of this file. The node depends on them.

### A2. Copy our code to our own folder
From the laptop, in the folder that contains the project:
```bash
rsync -av --exclude '.git' --exclude 'notebooks' --exclude '__pycache__' \
  Github/ <user>@<address>:~/malak_pid/
```
Re-run the same command after every change. On the robot, install missing packages for our user only:
```bash
pip install --user numpy pyyaml
```
If something needs `sudo` or changes his environment, ask him first.

### A3. Run
```bash
ssh <user>@<address>
cd ~/malak_pid
source /opt/ros/<distro>/setup.bash
# first: dry run (computes everything, publishes ZERO speed), watch the log
python3 ros/pid_node.py --ros-args -p path_file:=paths/line_1m.csv -p v_ref:=0.08 -p dry_run:=true
# then the real thing (remove dry_run)
python3 ros/pid_node.py --ros-args -p path_file:=paths/line_1m.csv -p v_ref:=0.08
```
Topic names and message types are parameters, so a mismatch is fixed without editing code:
`-p pose_topic:=/the_real_topic -p pose_type:=Odometry -p cmd_topic:=/the_real_cmd_topic -p cmd_stamped:=true`
(see the node's docstring for all parameters).

### A4. First tests (in this order)
1. **Wheels lifted off the ground.** Send a small command and check the directions: a positive `w` must make the right wheel faster than the left.
2. Someone holds the joystick / knows the stop method.
3. Short straight line (about 1 m) in a clear space, with a low `v_ref` (for example 0.08 m/s instead of 0.18).
4. Manual stop command, ready in another terminal:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
   ```
   (assumes the type is `Twist`; check in A1)
5. Then try the real path, then raise the speed.

### A5. Things to watch
- Others using the robot at the same time: two sources publishing to `/cmd_vel` will conflict.
- The Pi stops the robot if no command arrives for 0.5 s (from the notebook). The node must publish at least that often.

---

## Route B: our own Raspberry Pi 4 with a screen

Everything is done on the Pi itself with its keyboard and screen. No SSH needed.

### B0. What we are missing (TO CONFIRM)
- [ ] The missing cable / connection between the Pi and the robot's motor side. TO CONFIRM which one, and how the Pi talks to the motors (serial? the robot's existing driver?).
- [ ] Whether the Pi already has ROS 2 installed. TO CONFIRM; if not, it must be installed first, plus the robot's driver and `diff_drive_controller` setup from the advisor's workspace.
- [ ] Where `/robot_pose` comes from on this setup (camera, mocap, localisation node, odometry). TO CONFIRM: without a pose source the controller cannot run.

### B1. Get the code onto the Pi (pick one)
- **USB stick:** copy the project folder (without `.git` and `notebooks/`) to the stick, plug it into the Pi, copy it to `~/malak_pid/`.
- **GitHub:** push the repo from the laptop, then on the Pi: `git clone <repo-url> ~/malak_pid`. Later changes: `git pull`. Needs internet on the Pi.

### B2. Set up on the Pi
```bash
cd ~/malak_pid
python3 -m venv --system-site-packages .venv    # optional; keeps ROS packages visible
source .venv/bin/activate
pip install numpy pyyaml
```
(`--system-site-packages` matters so that ROS 2's Python packages stay importable.)

### B3. Check ROS and the topics (same commands as A1)
Run the commands from A1 in the Pi's terminal and fill in the "Results" section.

### B4. Run and test
Same as A3 and A4: run `python3 ros/pid_node.py` from the project root, wheels lifted first, then a short line, then the real path. Keep a way to cut power or stop the motors within reach.

### B5. Editing on the Pi
Small changes (gains in `config.yaml`) can be made directly on the Pi with `nano` or its text editor. If we also edit on the laptop, pick one place as the source of truth, or we will overwrite changes.

---

## What is missing in the folder (code side, both routes)

| # | Missing | Why | Status |
|---|---|---|---|
| 1 | `ros/pid_node.py`: a ROS 2 node | Subscribes to the pose, calls `controller.step(pose, t)` on a 30 Hz timer, publishes `Twist` on `/cmd_vel`. | **DONE**, tested in Docker against a fake robot (not on a real robot) |
| 2 | Pose conversion | `common/pose_utils.py`: `PoseStamped`, `Pose`, `PoseWithCovarianceStamped`, `Odometry` (quaternion to yaw) and `Pose2D`. Chosen with `-p pose_type:=...`. | **DONE**; which type the robot really uses is TO CONFIRM |
| 3 | Stale-pose watchdog and shutdown stop | Zero speed if no pose for 0.3 s (`pose_timeout`), on Ctrl+C, and after `max_time`. | **DONE**, tested in Docker |
| 4 | Done handling | Publishes zero and exits cleanly when `controller.done` is true. | **DONE**, tested in Docker |
| 5 | Path file + loader | `load_waypoints` / `path_from_file` in `common/paths.py`: CSV (`x,y` per line, `#` comments, optional header) or YAML. Example: `paths/line_1m.csv`. | **DONE**, tested; the real path file just has to follow this format |
| 6 | Frame match | The path coordinates must be in the same frame as `/robot_pose`. The node warns if the robot starts more than 0.5 m from the path start. `-p path_frame:=robot_start` shifts the path so it starts where the robot is (translation only, no rotation). | Mechanism **DONE**; which frame the real pose uses is TO CONFIRM |
| 7 | Real topic names / types | Parameters `pose_topic`, `pose_type`, `cmd_topic`, `cmd_stamped`. Defaults are placeholders: `/robot_pose` (PoseStamped), `/cmd_vel` (Twist). | TO CONFIRM on the robot |
| 8 | Real elapsed `dt` (optional) | `PIDPathTracker` uses a fixed `dt` (1/30 s) for its derivative and integral. Timer jitter on a Pi is probably fine at first. | Later improvement |
| 9 | Config update with measured values | Replace the "(assumed)" values with real measurements. Only matters for the simulator and for comparing sim to real. | After first tests |
| 10 | A lighter requirements file for the robot | `requirements-robot.txt` (numpy, pyyaml only). | **DONE** |

What is **not** missing: `controllers/pid.py` was not changed. The notebook, plotting and metrics are for the laptop.

Still not done: item 8 (real elapsed `dt`), item 9 (measured config values), and everything that needs the robot: the real topic names and types, the pose frame, the real path file.

---

## Testing without hardware (Docker)

Nothing needs to be installed on the laptop: a ROS 2 Humble image runs the node against a fake robot
(`ros/fake_robot.py`: the notebook's simulator behind `/cmd_vel` and `/robot_pose`).

```bash
# once (or after changing the Dockerfile). Docker Desktop must be running.
docker build -f docker/Dockerfile -t pid-ros .

# helper checks (path files, pose conversion) + the existing regression on all preset paths
docker run --rm -v "$PWD":/app pid-ros bash -c "python3 tests/test_helpers.py && python3 tests/run_all.py"

# end-to-end: the real node against the fake robot (goal reached, speed limits, stale-pose stop, dry run)
docker run --rm -v "$PWD":/app pid-ros python3 tests/ros_integration.py
```
`-v "$PWD":/app` uses the current files, so no rebuild is needed after editing code.

What this proves: the node's logic works with a real ROS 2 install (topics, timer, shutdown, watchdog).
What it does not prove: anything about the real robot (its message types, frame, latency, wheel behaviour).
If the robot's Pi runs another ROS 2 version, build with `--build-arg ROS_DISTRO=<that version>`.

---

## Results (fill in once we have access)

```
ROS_DISTRO:
/robot_pose message type:
/robot_pose rate (Hz):
/robot_pose frame / origin:
/cmd_vel message type:
Who else publishes to /cmd_vel:
Python version:
numpy / pyyaml installed?:
How the robot software is launched:
Emergency stop method:
```
