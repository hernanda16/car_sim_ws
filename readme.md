# 🚗 car_sim_ws

A ROS 2 Humble–based towing vehicle simulation using **Ignition Gazebo** and **ros2_control**, featuring pseudo-Ackermann steering and rear-wheel drive (RWD).

This project focuses on simulating an industrial towing/AGV vehicle that is **not a pure differential drive**, providing more realistic rear-wheel propulsion and front steering behavior.

---

# ✨ Features

* ✅ Modular URDF/Xacro
* ✅ ros2_control + Ignition Gazebo integration
* ✅ Rear-Wheel Drive (RWD)
* ✅ Pseudo Ackermann steering (Level 3)
* ✅ Differential rear wheel support
* ✅ RViz visualization launch
* ⚠️ Gazebo Classic launch retained for legacy use

---

# 📂 Launch Files

| Launch file          | Purpose                           |
| -------------------- | --------------------------------- |
| `all.launch.py`      | Gazebo Classic (legacy)           |
| `display.launch.py`  | RViz visualization (TF + URDF)    |
| `gazebo.launch.py`   | Experimental / currently ignored  |
| `ignition.launch.py` | **Main simulation (recommended)** |

---

# 🧰 Requirements

* Ubuntu 22.04
* ROS 2 Humble
* Ignition Gazebo
* ros2_control
* ign_ros2_control

Install dependencies:

```bash
sudo apt install \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ign-ros2-control
```

---

# 🚀 Running the Simulation

## 1️⃣ Build the workspace

```bash
cd ~/car_sim_ws
colcon build
```

---

## 2️⃣ Start the simulation

Use the provided helper script:

```bash
./run.sh
```

This will automatically configure the environment and launch the Ignition simulation.

---

# 👀 Visualization Only (RViz)

To visualize the robot model and TF tree in RViz, use the display launch provided in the package.

---

# 🔍 Check Controllers

After the simulation starts:

```bash
ros2 control list_controllers
```

Expected active controllers:

* `joint_state_broadcaster`
* `drive_controller`
* `steering_controller`

---

# ⚙️ Vehicle Parameters

```yaml
wheel_separation: 0.86
wheel_radius: 0.23
wheel_base: 1.01086
```

These parameters are used for:

* rear wheel differential computation
* steering angle calculation
* overall kinematic consistency

---

# 🧠 Control Model

## Rear wheel differential

[
v_L = v - \frac{ω \cdot wheel_separation}{2}
]

[
v_R = v + \frac{ω \cdot wheel_separation}{2}
]

---

## Steering (pseudo Ackermann)

[
\delta = atan\left(\frac{ω \cdot wheel_base}{v}\right)
]

---

# 🧪 Troubleshooting

## ❌ ign_ros2_control plugin not loading

Ensure the package is installed:

```bash
sudo apt install ros-humble-ign-ros2-control
```

Verify the library exists in:

```bash
/opt/ros/humble/lib/
```
