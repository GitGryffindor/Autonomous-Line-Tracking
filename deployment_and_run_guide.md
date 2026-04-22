# Deployment and Running Guide

Follow these exact steps to copy the code to your Waffle Pi and run the APF "Magnetic Fields" autonomous tracking system.

## 1. Copy Code to Pi (Run on Laptop)

Run this command from the repository root on your laptop to transfer the ROS2 package.  
*Note: Replace `waffle.local` with your Pi's actual IP if needed.*

```bash
rsync -avz --exclude '.git' /home/aashish/line_tracking/Autonomous-Line-Tracking/waffle_pi_lane_tracking/ waffle@waffle.local:~/Desktop/ROS2_workspace/src/waffle_pi_lane_tracking

scp -r waffle_pi_lane_tracking waffle@10.61.92.179:~/Desktop/ROS2_workspace/src/
```

---

## 2. Terminal Commands (On the Waffle Pi)

### **Terminal 1: Start Robot Drivers**

```bash
ssh waffle@waffle.local
ros2 launch turtlebot3_bringup robot.launch.py
```

### **Terminal 2: Start Camera**

```bash
ssh waffle@waffle.local
ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480
```

### **Terminal 3: Build & Run Autonomous Brain**

```bash
ssh waffle@waffle.local
cd ~/Desktop/ROS2_workspace
colcon build --packages-select waffle_pi_lane_tracking
source install/setup.bash
# Run with force markers enabled for RViz
ros2 launch waffle_pi_lane_tracking line_tracking.launch.py enable_rviz:=true
```

---

## 3. Terminal Commands (On your Laptop)

### **Terminal 4: Visualization (The "Eyes")**

Run this locally on your laptop to see the magnetic forces.  
*Ensure your laptop's workspace is sourced and on the same network.*

```bash
ros2 launch waffle_pi_lane_tracking line_tracking.launch.py rviz:=true
```

---

## **Command Summary Table**


| Component       | Device | Command                                                                         |
| --------------- | ------ | ------------------------------------------------------------------------------- |
| **Drivers**     | Pi     | `ros2 launch turtlebot3_bringup robot.launch.py`                                |
| **Camera**      | Pi     | `ros2 run camera_ros camera_node ...`                                           |
| **Brain (APF)** | Pi     | `ros2 launch waffle_pi_lane_tracking line_tracking.launch.py enable_rviz:=true` |
| **Eyes (RViz)** | Laptop | `ros2 launch waffle_pi_lane_tracking line_tracking.launch.py rviz:=true`        |


> [!TIP]
> You can tune the "Magnetic Pull" strength in Terminal 3 by adding:  
> `k_att:=0.005 k_rep:=0.40`