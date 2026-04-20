# Graph Report - /home/aashish/line_tracking/Autonomous-Line-Tracking  (2026-04-20)

## Corpus Check
- 11 files · ~16,383 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 61 nodes · 86 edges · 11 communities detected
- Extraction: 85% EXTRACTED · 15% INFERRED · 0% AMBIGUOUS · INFERRED: 13 edges (avg confidence: 0.73)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Community 0|Community 0]]
- [[_COMMUNITY_Community 1|Community 1]]
- [[_COMMUNITY_Community 2|Community 2]]
- [[_COMMUNITY_Community 3|Community 3]]
- [[_COMMUNITY_Community 4|Community 4]]
- [[_COMMUNITY_Community 5|Community 5]]
- [[_COMMUNITY_Community 6|Community 6]]
- [[_COMMUNITY_Community 7|Community 7]]
- [[_COMMUNITY_Community 8|Community 8]]
- [[_COMMUNITY_Community 9|Community 9]]
- [[_COMMUNITY_Community 10|Community 10]]

## God Nodes (most connected - your core abstractions)
1. `LineTrackingController` - 16 edges
2. `LaneFilter` - 9 edges
3. `DetectLane` - 9 edges
4. `CurveFit` - 7 edges
5. `ControlRobot` - 6 edges
6. `BirdsEyeView` - 6 edges
7. `main()` - 3 edges
8. `main()` - 3 edges
9. `sanitize()` - 3 edges
10. `main()` - 3 edges

## Surprising Connections (you probably didn't know these)
- `LaneFilter` --uses--> `DetectLane`  [INFERRED]
  /home/aashish/line_tracking/Autonomous-Line-Tracking/waffle_pi_lane_tracking/waffle_pi_lane_tracking/lane_filter.py → /home/aashish/line_tracking/Autonomous-Line-Tracking/waffle_pi_lane_tracking/waffle_pi_lane_tracking/detect_lane.py
- `DetectLane` --uses--> `CurveFit`  [INFERRED]
  /home/aashish/line_tracking/Autonomous-Line-Tracking/waffle_pi_lane_tracking/waffle_pi_lane_tracking/detect_lane.py → /home/aashish/line_tracking/Autonomous-Line-Tracking/waffle_pi_lane_tracking/waffle_pi_lane_tracking/curve_fit.py
- `DetectLane` --uses--> `BirdsEyeView`  [INFERRED]
  /home/aashish/line_tracking/Autonomous-Line-Tracking/waffle_pi_lane_tracking/waffle_pi_lane_tracking/detect_lane.py → /home/aashish/line_tracking/Autonomous-Line-Tracking/waffle_pi_lane_tracking/waffle_pi_lane_tracking/birds_eye_view.py

## Communities

### Community 0 - "Community 0"
Cohesion: 0.29
Nodes (2): LineTrackingController, main()

### Community 1 - "Community 1"
Cohesion: 0.25
Nodes (4): Enum, quaternion_to_yaw(), RobotState, sanitize()

### Community 2 - "Community 2"
Cohesion: 0.39
Nodes (1): LaneFilter

### Community 3 - "Community 3"
Cohesion: 0.29
Nodes (4): generate_launch_description(), generate_launch_description(), generate_launch_description(), Node

### Community 4 - "Community 4"
Cohesion: 0.47
Nodes (2): ControlRobot, main()

### Community 5 - "Community 5"
Cohesion: 0.4
Nodes (1): BirdsEyeView

### Community 6 - "Community 6"
Cohesion: 0.5
Nodes (0): 

### Community 7 - "Community 7"
Cohesion: 0.83
Nodes (2): DetectLane, main()

### Community 8 - "Community 8"
Cohesion: 0.5
Nodes (1): CurveFit

### Community 9 - "Community 9"
Cohesion: 1.0
Nodes (0): 

### Community 10 - "Community 10"
Cohesion: 1.0
Nodes (0): 

## Knowledge Gaps
- **Thin community `Community 9`** (1 nodes): `setup.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 10`** (1 nodes): `__init__.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `DetectLane` connect `Community 7` to `Community 2`, `Community 3`, `Community 5`, `Community 6`, `Community 8`?**
  _High betweenness centrality (0.528) - this node is a cross-community bridge._
- **Why does `LineTrackingController` connect `Community 0` to `Community 1`, `Community 3`?**
  _High betweenness centrality (0.511) - this node is a cross-community bridge._
- **Why does `LaneFilter` connect `Community 2` to `Community 5`, `Community 7`?**
  _High betweenness centrality (0.185) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `LaneFilter` (e.g. with `DetectLane` and `.__init__()`) actually correct?**
  _`LaneFilter` has 2 INFERRED edges - model-reasoned connections that need verification._
- **Are the 3 inferred relationships involving `DetectLane` (e.g. with `CurveFit` and `BirdsEyeView`) actually correct?**
  _`DetectLane` has 3 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `CurveFit` (e.g. with `DetectLane` and `.__init__()`) actually correct?**
  _`CurveFit` has 2 INFERRED edges - model-reasoned connections that need verification._