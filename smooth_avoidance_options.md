# Smooth Obstacle Avoidance — Upgrade Options

This document explores methods to eliminate jerky movements during obstacle avoidance by transitioning from discrete state jumps to continuous velocity control.

## Problem: Why the Current Approach Is Jerky

The current state machine creates **three hard velocity discontinuities** during every obstacle bypass:

```mermaid
sequenceDiagram
    participant R as Robot
    participant V as Velocity

    Note over R,V: FOLLOWING_LINE
    R->>V: vx=0.15, wz=proportional

    Note over R,V: ⚠️ Obstacle detected!
    R->>V: vx=0.0, wz=0.0 (FULL STOP)
    Note right of V: 💥 Jerk #1

    Note over R,V: EVADING
    R->>V: vx=0.05, wz=-0.50 (hard spin)
    Note right of V: 💥 Jerk #2

    Note over R,V: CLEARING
    R->>V: vx=0.12, wz=0.0 (dead straight)
    Note right of V: 💥 Jerk #3

    Note over R,V: ARC_RECOVERY
    R->>V: vx=0.10, wz=+0.45 (fixed arc)
    Note right of V: 💥 Jerk #4
```

Each state transition is an **instantaneous velocity jump** — the wheels go from one speed to a completely different one in a single 50ms tick. This causes physical jerking, wheel slip, and unstable sensor readings.

---

## Option A: Velocity Ramping (Easiest)

**Keep the existing state machine, but smooth velocity transitions using exponential moving average (EMA).**

### How It Works

```mermaid
flowchart LR
    subgraph StateMachine["State Machine (unchanged)"]
        S1[FOLLOWING_LINE] --> S2[EVADING]
        S2 --> S3[CLEARING]
        S3 --> S4[ARC_RECOVERY]
        S4 --> S1
    end

    subgraph Smoother["Velocity Smoother (NEW)"]
        T[Target vx, wz] --> EMA["EMA Filter<br/>α = 0.15"]
        EMA --> CMD["/cmd_vel"]
    end

    StateMachine --> |"target velocities"| Smoother
```

### Code Changes Sketch

Add a velocity smoother that sits between the state machine output and the `/cmd_vel` publisher:

```python
# New instance variables in __init__:
self.smooth_vx = 0.0
self.smooth_wz = 0.0
self.alpha     = 0.15   # smoothing factor (lower = smoother, 0.10–0.20 recommended)

# Replace the _vel() method:
def _vel(self, vx=0.0, wz=0.0):
    """Publish smoothed velocity — ramps toward target instead of jumping."""
    self.smooth_vx += self.alpha * (vx - self.smooth_vx)
    self.smooth_wz += self.alpha * (wz - self.smooth_wz)
    msg = Twist()
    msg.linear.x  = float(self.smooth_vx)
    msg.angular.z = float(self.smooth_wz)
    self.cmd_pub.publish(msg)
```

### Pros & Cons

| Aspect | Rating |
|--------|--------|
| Implementation effort | ⭐ Very easy (~10 lines changed) |
| Smoothness | ⭐⭐ Good — removes velocity jumps |
| Risk of breaking existing logic | Very low |

---

## Option B: Curved Bypass Path (Moderate)

**Replace the spin-straight-arc maneuver with a single continuous D-shaped curve.**

### How It Works

```mermaid
flowchart TD
    subgraph Current["Current: 3-Phase Maneuver"]
        direction LR
        E1["EVADING<br/>Spin 90° right<br/>wz = -0.50"] --> E2["CLEARING<br/>Drive straight<br/>wz = 0.00"]
        E2 --> E3["ARC_RECOVERY<br/>Arc left<br/>wz = +0.45"]
    end

    subgraph Smooth["Proposed: Single Curved Bypass"]
        direction LR
        C1["BYPASSING<br/>Sinusoidal steering<br/>wz = f(progress)"]
    end

    Current -.-> |"Replace with"| Smooth
```

The angular velocity follows a **sinusoidal profile** instead of discrete steps:

```python
# Sinusoidal steering: curves right → straightens → curves left
# progress 0.0–1.0
wz = -max_turn * math.sin(progress * 2.0 * math.pi - math.pi / 3.0)
```

### State Diagram (Simplified)

```mermaid
stateDiagram-v2
    [*] --> FOLLOWING_LINE: Node Start

    state FOLLOWING_LINE {
        direction LR
        APF: Proportional Steering
        Detect: LiDAR Front < 0.35m
    }

    FOLLOWING_LINE --> BYPASSING: Obstacle Detected

    state BYPASSING {
        direction LR
        SineCurve: "wz = -A·sin(2π·t/T)"
        CameraWatch: Camera scans for line
    }

    BYPASSING --> FOLLOWING_LINE: "Line Found (progress > 50%)"
    BYPASSING --> STUCK: "Timeout (> 12s)"
```

---

## Option C: Full APF Blending (Best Smoothness)

**Extend the existing APF line-attraction to include LiDAR-based obstacle repulsion.**

### How It Works

```mermaid
flowchart TD
    subgraph Inputs["Sensor Inputs"]
        CAM["📷 Camera<br/>Line centroid error"]
        LIDAR["📡 LiDAR<br/>Obstacle distances"]
    end

    subgraph APF["Artificial Potential Field"]
        ATT["Attraction Force<br/>F_att = -k_att × error"]
        REP["Repulsion Force<br/>F_rep = k_rep / d²"]
        BLEND["Σ Blend<br/>wz = F_att + F_rep_lateral"]
    end

    CAM --> ATT
    LIDAR --> REP
    ATT --> BLEND
    REP --> BLEND
    BLEND --> VEL[Output: /cmd_vel]
```

### Force Diagram

```mermaid
flowchart LR
    subgraph Forces["Force Diagram (top-down)"]
        direction TB
        LINE["← F_att (line pull)"]
        OBS["→ F_rep (obstacle push)"]
        NET["↗ F_net (resultant)"]
    end
```

### Pros & Cons

| Aspect | Rating |
|--------|--------|
| Smoothness | ⭐⭐⭐⭐ Best — completely continuous |
| Implementation effort | ⭐⭐⭐ Significant (~100+ lines) |
| Risk | Higher — fundamentally different approach |

---

## Comparison Summary

| Feature | Option A: Ramping | Option B: Curved Bypass | Option C: Full APF |
|---------|-------------------|-------------------------|--------------------|
| **Difficulty** | Easy | Moderate | Hard |
| **Smoothness** | Good | Excellent | Best |
| **Keeps SM?** | ✅ Yes | Partially | ❌ No |
| **Recommended** | Quick win | Balanced | Research/Pro |
