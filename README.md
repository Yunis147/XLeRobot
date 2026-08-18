# XLeRobot — Complete SLAM & Navigation Guide

---

## 1. Initial Setup (One Time)

### 1.1 Create workspace and clone repo

```bash
mkdir -p ~/xlerobot/src
cd ~/xlerobot/src
git clone https://github.com/KMTI-ROBOPARADIGM/mobile-manipulator.git
cd mobile-manipulator
git checkout xlerobot
```

Note: the folder is named `mobile-manipulator` (the repo name) — we only switched **branches** to get the XLeRobot code, we didn't clone a different repo. Every path below reflects this.

### 1.2 Build the workspace

```bash
cd ~/xlerobot
colcon build
source install/setup.bash
```

Add this to `~/.bashrc` so you don't have to source manually every time:

```bash
echo "source ~/xlerobot/install/setup.bash" >> ~/.bashrc
```

---

## 2. Building a Map (SLAM)

You need **three terminals** on the Pi.

**Terminal 1 — Odometry (motor control + wheel odometry):**
```bash
cd ~/xlerobot/src/mobile-manipulator/nav2/nav2
python3 odom.py
```
Wait until it prints position data before continuing.

**Terminal 2 — Keyboard teleop (to drive the robot manually while mapping):**
```bash
cd ~/xlerobot/src/mobile-manipulator/nav2/nav2
python3 teleop.py
```

**Terminal 3 — SLAM (builds the map from LiDAR while you drive):**
```bash
ros2 launch nav2 xle.slam.py
```

### 2.1 Visualizing in RViz while building the map

On your laptop:
```bash
export ROS_DOMAIN_ID=<same number as Pi>
rviz2
```

First, set **Global Options → Fixed Frame** (top of the Displays panel) to `map`.

Then add these displays one at a time — click **Add** at the bottom-left of the Displays panel each time:

1. **Map**
   - Add → By display type → `Map`
   - Set **Topic** to `/map`
   - Set **Durability Policy** to `Transient Local`
   - *Why Transient Local matters:* the map is only re-published when it changes, not on a continuous stream. If RViz subscribes with the default "Volatile" durability, it can miss the last published map and show nothing until the next update. Transient Local makes RViz immediately fetch the most recently published map the moment it subscribes.

2. **TF**
   - Add → By display type → `TF`
   - Shows every coordinate frame (`map`, `odom`, `base_footprint`, `base_link`, `laser`) and the live connections between them — a quick visual check that the whole TF chain is alive and nothing is disconnected.

3. **LaserScan**
   - Add → By topic → `/scan` → `LaserScan`
   - Shows the live LiDAR hits as dots around the robot. As you drive, these dots should trace out the same walls the map is drawing — if they consistently overlay newly-drawn map cells cleanly, mapping is going well.

Now drive the robot slowly around the whole space using `teleop.py` (WASD keys) until the map is fully covered — go down every corridor, into every room, and ideally close the loop (return near your starting point) so SLAM can correct any accumulated drift.

### 2.2 Saving the map

Once you're happy with the map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/xlerobot/src/mobile-manipulator/nav2/nav2/map/xle_room_map
```
This creates `xle_room_map.yaml` and `xle_room_map.pgm`. This is the file `xle.nav2.py` loads for navigation.

---

## 3. Navigating the Map

**Terminal 1 — Odometry (always first):**
```bash
cd ~/xlerobot/src/mobile-manipulator/nav2/nav2
python3 odom.py
```

**Terminal 2 — Nav2 (planner, controller, AMCL, costmaps):**
```bash
ros2 launch nav2 xle.nav2.py
```

### 3.1 About ROS_DOMAIN_ID

ROS2 nodes only discover each other if they share the same `ROS_DOMAIN_ID`. This is how your laptop's RViz can see topics being published on the Pi over the same network. Check the Pi's domain ID:
```bash
echo $ROS_DOMAIN_ID
```
Set the exact same value on your laptop before launching RViz, or nothing will show up (blank displays, no topics found):
```bash
export ROS_DOMAIN_ID=<same number as Pi>
```

### 3.2 Visualizing in RViz while navigating

Navigation has more moving parts than SLAM — the saved map, two live costmaps, the planned path, and the localization estimate. Add these one at a time.

On your laptop:
```bash
rviz2
```
Set **Global Options → Fixed Frame** to `map`.

1. **Map — the saved map**
   - Add → By display type → `Map`
   - Topic: `/map`
   - Durability Policy: `Transient Local`
   - Same static map from SLAM, now loaded for localization.

2. **Map — Global Costmap**
   - Add → By display type → `Map` (yes, add a second, separate Map display)
   - Topic: `/global_costmap/costmap`
   - Shows cost values across the whole map that the path planner uses — you'll see a colored gradient inflating outward from every wall.

3. **Map — Local Costmap**
   - Add → By display type → `Map` (a third Map display)
   - Topic: `/local_costmap/costmap`
   - A small rolling window centered on the robot, rebuilt live from current LiDAR data. This is what actually drives real-time obstacle avoidance and reacts to things not in the saved map.

4. **TF**
   - Add → By display type → `TF`
   - Same as before — confirms the full frame chain is alive.

5. **LaserScan**
   - Add → By topic → `/scan` → `LaserScan`
   - Live LiDAR points.

6. **Path**
   - Add → By topic → `/plan` → `Path`
   - Draws the planned route from robot to goal as a line. Appears once you send a goal.

7. **Pose — AMCL estimate**
   - Add → By topic → `/amcl_pose` → `PoseWithCovariance` (labeled `Pose` in some RViz versions)
   - Shows the robot's estimated position and facing direction as an arrow, with an ellipse showing how confident AMCL currently is.

**Optional, useful for confirming localization has converged:**
- Add → By topic → `/particlecloud` → `PoseArray`
- Shows every individual AMCL particle as a tiny arrow. Right after you set the initial pose these are scattered; watching them collapse into a tight cluster is a clear visual sign that AMCL has locked on.

### 3.3 Sending a goal

1. Click **2D Pose Estimate** in the RViz toolbar, then click-and-drag on the map at the robot's actual physical location — the drag direction sets the initial facing direction.
2. Wait a few seconds. The LaserScan dots should snap onto the map's walls, and (if added) the particle cloud should tighten into a small cluster. This confirms AMCL has localized.
3. Click **2D Goal Pose** (or **Nav2 Goal** if using the Nav2 panel) in the toolbar, then click-and-drag at your destination — the drag direction sets the orientation you want the robot to end up facing.
4. The `Path` display shows the planned route, and the robot starts moving.

### 3.4 Noting down waypoints (e.g. table positions)

Two good methods, depending on what you need.

**Method 1 — Publish Point (position only, fast, robot doesn't move)**

Use this when you just want XY coordinates of a spot on the map without driving there. No orientation is captured.

1. In RViz, click **Publish Point** in the toolbar.
2. Click on the map at the location you want.
3. On the Pi (or laptop, same `ROS_DOMAIN_ID`), run:
```bash
ros2 topic echo /clicked_point --once
```
4. Note the `x` and `y` values printed.

**Method 2 — Drive there and read AMCL pose (position + orientation, most accurate)**

Use this when the exact facing direction matters too — e.g. a table the robot needs to approach at a specific angle.

1. With `odom.py` and `xle.nav2.py` running and AMCL localized in RViz (per 3.3), open a new terminal and run teleop:
```bash
cd ~/xlerobot/src/mobile-manipulator/nav2/nav2
python3 teleop.py
```
2. Drive the robot to the exact spot, facing the exact direction you want it to end up at.
3. Once positioned, run:
```bash
ros2 topic echo /amcl_pose --once
```
4. Note `position.x`, `position.y`, and the orientation quaternion's `z` and `w`.
5. Convert to yaw:
```
yaw_radians = 2 * atan2(z, w)
```

Method 2 is more reliable for waypoints where facing direction matters, since you're recording the robot's actual real-world pose rather than estimating an angle from a 2D click on the map.

---

## 4. How SLAM and Nav2 Actually Work

### 4.1 What the LiDAR gives you

The RPLidar A1 spins and measures distance to whatever it hits at each angle, publishing a `sensor_msgs/LaserScan` message about 10 times per second. Each message contains:

- `angle_min` / `angle_max` — the sweep range (for a 360° lidar, roughly -π to π)
- `angle_increment` — the angular gap between each reading (e.g. 1°)
- `ranges[]` — an array of distances, one per angle step. If a beam hits nothing within range, that value is `inf`
- `range_min` / `range_max` — valid distance bounds (readings outside this are discarded)

So essentially every scan is: *"at angle X, there's something Y meters away"* repeated for every angle around the robot.

### 4.2 How SLAM builds the map

SLAM = **S**imultaneous **L**ocalization **A**nd **M**apping — it solves two problems at once: "where am I?" and "what does the environment look like?" using only the LiDAR and odometry.

The map is an **occupancy grid** — the world divided into small square cells (we use 5cm resolution). Each cell holds a probability: occupied, free, or unknown.

How it fills in as you drive:
1. Every laser scan gives distances at every angle from the robot's current (estimated) position
2. For each beam, SLAM ray-traces from the robot to the hit point: every cell along that ray is marked as **free space** (the laser passed through it), and the cell at the hit point is marked **occupied** (something is there)
3. As you drive further and see the same walls again from a different angle, SLAM performs **scan matching** — comparing the new scan against the map built so far to figure out exactly how much the robot has moved and rotated since the last scan. This corrects small drift errors from odometry
4. If you drive in a loop and come back near your starting point, SLAM recognizes the revisited area (**loop closure**) and retroactively corrects the accumulated position error across the whole path, "snapping" the map into a consistent shape

This is why driving slowly and covering every area (including returning to a loop) gives a much cleaner map than driving fast in a straight line once.

### 4.3 How the robot knows its position and orientation (localization)

Once a map exists, we switch from SLAM to **AMCL** (Adaptive Monte Carlo Localization) for navigation. AMCL uses a **particle filter**:

1. AMCL starts with thousands of "particles" — each one is a guess of where the robot might be (x, y, theta). Initially these are spread around wherever you clicked **2D Pose Estimate**
2. As the robot moves (tracked via odometry), every particle moves by the same amount, with a bit of random noise added (real motion isn't perfectly predictable)
3. When a new laser scan arrives, AMCL checks: *"if I were actually at this particle's position, would my LiDAR see what it's currently seeing, given the saved map?"* Particles whose predicted scan matches the real scan well get a high weight; particles that don't match get a low weight
4. **Resampling**: particles with low weight are discarded, particles with high weight are duplicated (with slight variation). Over a few iterations, all particles converge tightly around the true position
5. The published `/amcl_pose` is essentially the weighted average of all surviving particles

AMCL publishes the **`map → odom`** transform — a correction offset. Combined with odom's own **`odom → base_footprint`** transform (from wheel encoders), you get the robot's true position in the map frame.

### 4.4 How the robot knows how far and which way it moved (odometry)

This is purely mechanical, computed in `odom.py`, with no LiDAR involved:

1. Each of the 3 omni wheels has a servo that reports its current rotational position (0–4096 ticks per revolution)
2. Every update cycle (30Hz), we read all 3 positions and compute the **delta** (how much each wheel turned) since the last reading
3. Using the known **omni-wheel kinematics** (wheel mounting angles: 60°, 180°, 300°), these 3 individual wheel deltas are combined mathematically to solve for how far the robot's center moved in X, Y, and how much it rotated (theta) — this is the reverse of how a joystick command gets split into 3 wheel speeds
4. These small deltas are added up continuously (**dead reckoning**) to track the running total position: `self.x`, `self.y`, `self.theta`

The catch: dead reckoning **drifts**. Small errors (wheel slip, rounding) accumulate over time and the estimate slowly diverges from reality. This is exactly why AMCL exists — it periodically corrects this drift using the LiDAR's view of the actual environment, which doesn't drift.

### 4.5 How Nav2 avoids collisions

Nav2 uses two **costmaps** — occupancy grids where each cell has a "cost" value instead of just occupied/free:

- **Global costmap** — covers the whole saved map, used by the path planner to find a route from robot to goal that avoids walls
- **Local costmap** — a small rolling window (3m × 3m in our case) centered on the robot, rebuilt continuously from live LiDAR data. This is what makes the robot react to obstacles that aren't in the saved map (furniture, people, etc.)

Each costmap is built from **layers**:
- **Static layer** — the saved map (walls) — global costmap only
- **Obstacle/voxel layer** — marks cells hit by the current LiDAR scan as occupied in real time
- **Inflation layer** — adds a gradient of cost around every obstacle, decaying outward. This is what keeps the robot from hugging walls — it's penalized for getting close, not just forbidden from touching

**The pipeline for one navigation cycle:**
1. **SmacPlanner2D** (global planner) searches the global costmap for the lowest-cost path from current position to goal — this happens once (and again if a big detour is needed)
2. **MPPI** (local controller) takes that path and, ~20 times a second, samples thousands of possible short-term trajectories the robot could take, scores each one using the local costmap plus the "critics" (see below), and picks the best-scoring one
3. That trajectory becomes an actual velocity command sent to `/cmd_vel`, which `odom.py` converts into individual wheel speeds

---

## 5. MPPI Critics — What They Are and What We Use

Each **critic** is a scoring function. MPPI generates thousands of random candidate trajectories every cycle; each critic adds or subtracts cost from every trajectory based on some property (does it hit an obstacle? does it point toward the goal? etc). The trajectory with the lowest total cost across all critics wins and becomes the output velocity.

### 5.1 Critics we currently use

| Critic | What it does | Our weight / setting |
|---|---|---|
| **ConstraintCritic** | Penalizes trajectories that would exceed the robot's velocity/acceleration limits | weight 4.0 |
| **GoalCritic** | Pulls the robot toward the goal's **XY position**. Only active once within `threshold_to_consider` of goal | weight 5.0, threshold 1.5m |
| **GoalAngleCritic** | Pulls the robot to face the goal's **yaw** (orientation). Only active within `threshold_to_consider` | weight 6.0, threshold 0.5m |
| **CostCritic** | Penalizes trajectories that pass through costed (near-obstacle) space, using the local costmap | weight 3.81, considers full footprint |
| **PathAlignCritic** | Keeps the robot's trajectory close to the globally planned path | weight 14.0, `use_path_orientations: false` |
| **PathFollowCritic** | Actively drives the robot forward along the path (this is the main "go" force) | weight 5.0 |
| **PathAngleCritic** | Penalizes trajectories at a sharp angle to the path direction, `mode: 2` uses the planner's own orientation data instead of just the path shape | weight 2.0 |

### 5.2 The two settings that were silently breaking orientation

**`use_final_approach_orientation` (in SmacPlanner2D / planner_server):**
- `true` → the last pose of the planned path gets the **approach tangent** — i.e., whatever direction the robot happened to be traveling as it arrived, NOT the orientation you set with the RViz goal arrow
- `false` → the last pose gets the **actual goal orientation** you specified in RViz

We had this set to `true`, so MPPI's `GoalAngleCritic` was chasing a goal orientation that wasn't the one we actually wanted. **Must be `false`.**

**`use_path_orientations` (inside PathAlignCritic):**
- `true` → tries to match the robot's heading to the orientation of *every intermediate pose* along the path, not just the final goal
- SmacPlanner2D's 2D mode gives all intermediate poses an **identity orientation (yaw = 0)** — it only computes real orientations for Hybrid/Lattice planners, not the 2D grid planner we use
- With this `true`, `PathAlignCritic` was constantly fighting to keep the robot facing map "east" (yaw=0) for the entire journey, regardless of the actual direction of travel → this is what caused the continuous rotation while driving to a goal

**Must be `false`** unless you switch to a Hybrid/Lattice planner that actually computes intermediate orientations.

### 5.3 Critics we tried and removed

| Critic | Why we don't use it |
|---|---|
| **TwirlingCritic** | Penalizes *all* angular velocity to stop random spinning. Sounds useful, but it directly fights `GoalAngleCritic`'s attempt to rotate the robot at the goal — on our omni robot this suppressed the final rotation almost entirely, even at high `GoalAngleCritic` weights |
| **PreferForwardCritic** | Penalizes any backward motion, biasing the robot to always face its direction of travel. For a holonomic (omni) robot this fights natural strafing and caused unnecessary rotation just to "face forward" while sliding sideways |

### 5.4 Other critics available (not currently used, but could help in specific cases)

| Critic | What it does | When you might want it |
|---|---|---|
| **ObstaclesCritic** | Alternative to CostCritic — computes obstacle distance directly rather than via costmap value, with separate `repulsion_weight` (soft avoidance) and `critical_weight` (hard avoidance near collision) | If you want finer control over "how much the robot avoids getting near things" vs "how hard it avoids actually hitting things" as two separate tunable numbers |
| **TwirlingCritic** | See above | If you have a diff-drive or the omni base and find it spins excessively *during transit* (not at the goal) — use a very low weight (1.0–2.0) only, never combine with high GoalAngleCritic demands |
| **VelocityDeadbandCritic** | Penalizes commanded velocities that fall below a deadband threshold — useful when your motors don't respond well to very small commands (motor "won't move" below some speed) | If you notice the robot twitching or stalling at very low speeds near the goal |
| **PreferForwardCritic** | See above | Only for differential-drive robots where you specifically want to discourage reversing (e.g. robot has no rear sensors) |

---

## 6. Quick Reference — Diagnostic Commands

```bash
# Current robot position/orientation as AMCL sees it
ros2 topic echo /amcl_pose --once

# What orientation the last goal actually requested
ros2 topic echo /goal_pose --once

# Check the planned path's final orientation
ros2 topic echo /plan --once | tail -20

# Watch live velocity commands
ros2 topic echo /cmd_vel

# Confirm which planner/controller plugins are actually loaded
ros2 param get /planner_server GridBased.plugin
ros2 param get /controller_server FollowPath.plugin

# Get x,y of a point by clicking on the map in RViz
# (click "Publish Point" in RViz toolbar, click the map, then:)
ros2 topic echo /clicked_point --once
```

**Converting quaternion to yaw (degrees):**
```
yaw_radians = 2 * atan2(z, w)
yaw_degrees = yaw_radians * 180 / pi
```
