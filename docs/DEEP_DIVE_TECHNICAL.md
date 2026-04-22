# Deep Dive: Technical Implementation & Mathematical Foundations

This document provides a low-level technical analysis of the algorithms and mathematical derivations used in the **Autonomous Line Tracking** project.

---

## 1. Control Theory: The Sinusoidal Bypass Logic

### The Objective

To move the robot from a line, around an obstacle, and back to the line in a smooth, continuous path. We choose a **Sinusoidal Velocity Profile** for the angular velocity (`w_z`) because it ensures `C_1` continuity (smooth acceleration/deceleration).

### Mathematical Derivation of Amplitude (`A`)

Given a target clearance angle `theta` (calculated from LiDAR geometry) and a maneuver time `T`, we define the angular velocity as:

```text
w_z(t) = -A * sin( (2 * pi * t) / T )
```

The change in heading (`delta_psi`) over the first half of the maneuver (`t = 0` to `t = T/2`) must equal `theta` to steer the robot away from the obstacle.

1. **Integrate `w_z` to find heading `psi(t)**`:

```text
psi(t) = Integral[ -A * sin( (2 * pi * tau) / T ) d_tau ] from 0 to t
psi(t) = [ (A * T) / (2 * pi) * cos( (2 * pi * tau) / T ) ] from 0 to t
psi(t) = (A * T) / (2 * pi) * ( cos( (2 * pi * t) / T ) - 1 )
```

1. **Evaluate at `t = T/2**`:

```text
psi(T/2) = (A * T) / (2 * pi) * ( cos(pi) - 1 )
psi(T/2) = (A * T) / (2 * pi) * ( -1 - 1 )
psi(T/2) = -(A * T) / pi
```

1. **Solve for `A**`:
  Since the required heading change is `theta` (magnitude):

```text
|psi(T/2)| = theta
(A * T) / pi = theta
A = (theta * pi) / T
```

**Implementation:**  
In `line_tracking.py`, this is precisely implemented as `amp_geo = theta * math.pi / self.bypass_time`.

---

## 2. Perception: SVD-Based Width Measurement

When the LiDAR detects points in the front cone, we treat them as a matrix $P \in \mathbb{R}^{N \times 2}$.

### Step 1: RANSAC Outlier Removal

LiDAR can catch "noise" (particles in the air or reflections). RANSAC (Random Sample Consensus) iteratively selects two points, fits a line, and counts "inliers" (points within 1.5cm of the line). The best line is kept, effectively isolating the **flat face** of the box.

### Step 2: Singular Value Decomposition (SVD)

We compute the centroid `P_avg` and center the points: `P' = P - P_avg`.  
We then perform SVD: `P' = U S V^T`.

- `**V_0` (First Principal Component)**: The vector pointing along the longest axis of the points (the face width).
- `**V_1` (Second Principal Component)**: The vector perpendicular to the face (the normal).

### Step 3: Projection for Width

We project all points onto the face direction `V_0`:

```text
projection_i = centered_point_i . V_0
```

The width of the obstacle is then:

```text
Width = Max(projections) - Min(projections)
```

This allows the robot to handle boxes of any size and orientation dynamically.

---

## 3. Vision: Advanced HSV Filtering

### HSV vs BGR

Standard BGR (Blue-Green-Red) is highly sensitive to light intensity. A shadow can change "Yellow" from `(0, 255, 255)` to `(0, 100, 100)`, breaking detection.  
**HSV** separates:

- **Hue (H)**: The "color" (fixed for yellow).
- **Saturation (S)**: The "vibrancy".
- **Value (V)**: The "brightness".

By setting a wide range for **Value** (e.g., 70-255), the robot remains functional in both direct sunlight and indoor shadows.

### Morphological Operations

To ensure the centroid calculation is stable, we apply:

1. **MORPH_OPEN (Erosion then Dilation)**: Removes small white noise (speckles).
2. **MORPH_CLOSE (Dilation then Erosion)**: Fills small black holes inside the yellow line (caused by tape reflection).

---

## 4. State Machine Architecture Details

### Transition Logic: The "Rescue" Condition

One of the most robust features is the **Rescue Area** condition. In the `BYPASSING` state, the robot is "blind" to the line for the first 50% of the movement. However, if it sees a large concentration of yellow pixels (`line_area > 3000`), it triggers a rescue:

- **Condition**: `progress > 0.5` AND `line_area > RESCUE_LIMIT`.
- **Reasoning**: This prevents the robot from completing a redundant arc if it has already cleared the box and returned to the line early.

### The "Stuck" Guard

If the robot remains in `BYPASSING` for more than 12 seconds without finding the line, it assumes it has missed the path or is boxed in.

- **Action**: Reverse at -0.1 m/s for 1.5 seconds and trigger a fresh LiDAR scan for a new avoidance angle.

---

## 5. Technical Specification (TurtleBot3 Waffle Pi)

- **LiDAR Scan (LDS-01)**: 360 points at 5Hz. We capture the "Front Cone" by stitching indices `[0:20]` and `[340:359]`.
- **IMU/Odometry**: Used to convert Quaternions into **Euler Yaw** (`psi`) for the geometry calculations.
- **Camera Pipeline**: Raspberry Pi Camera V2 streaming at 320x240 @ 30FPS for low-latency control loops.

---

## 6. Code Architecture: Key Components

The core logic resides in `line_tracking.py`, structured as a ROS2 Lifecycle-style node.

### Principal Functions:


| Function            | Purpose          | Logic Context                                                                                |
| ------------------- | ---------------- | -------------------------------------------------------------------------------------------- |
| `_img_cb`           | Image Subscriber | Decodes NV21/NV12 raw camera streams into OpenCV mats.                                       |
| `_detect_line`      | Vision Engine    | Applies HSV mask, finds the largest contour, and returns the X-centroid.                     |
| `_scan_cb`          | LiDAR Processing | Extracts the front cone, calculates `front_dist`, and runs SVD/RANSAC analysis.              |
| `_loop`             | 20Hz Heartbeat   | The main FSM (Finite State Machine) runner. Checks safety zones and selects the control law. |
| `_go`               | State Transition | Manages "garbage collection" (clearing geometry data) when entering/exiting bypass.          |
| `measure_box_width` | Geometry Engine  | Uses SVD to calculate physical width/distance for obstacle arcs.                             |


---

## 7. Tuning & Parameters Guide

To change the robot's behavior, modify these parameters in the node or through `ros2 param set`:


| Parameter       | Default | Effect                                                                                               |
| --------------- | ------- | ---------------------------------------------------------------------------------------------------- |
| `k_att`         | `0.010` | **Line Aggression**: Higher values make the robot snap to the line faster but can cause oscillation. |
| `follow_speed`  | `0.15`  | **Cruise Speed**: The m/s speed on straight line segments.                                           |
| `zone2_dist`    | `0.75m` | **Bypass Trigger**: How far away the robot starts the sinusoidal curve.                              |
| `bypass_time`   | `6.0s`  | **Curve Duration**: Total seconds spent in the avoidance maneuver.                                   |
| `SAFETY_MARGIN` | `0.30m` | **Lateral Buffer**: Extra distance added to the box width to ensure the robot clears the side.       |


---

## 8. Debugging & Visualization

The system includes a custom **RViz Marker Array** publisher (`/visualization_marker_array`).

- **Blue Arrow**: Represents the "Line Pull" (Attractive force).
- **Red Arrow**: Represents the "Obstacle Push" (if implemented as repulsive).
- **Green Arrow**: Represents the **Resultant Command** being sent to the motors.

By enabling `enable_rviz: True`, you can see the robot's "thoughts" in real-time within the ROS2 rviz environment.

---