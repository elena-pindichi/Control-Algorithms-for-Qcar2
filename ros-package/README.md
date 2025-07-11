# 🚗 Connecting to the QCar via ROS 2 (Humble)

This guide walks you through connecting your ROS 2 workspace to a QCar platform using the QTM camera system and launching the vehicle nodes.

---

## 🔧 Prerequisites
- **ROS 2 Humble** installed on both your development machine and the QCar
- **SSH access** to the QCar
- **QTM camera software** running

---

## 🧭 Step-by-Step Instructions

### 🖥️ Terminal 1: Connect to QTM
Open a terminal in your ROS 2 workspace and connect to the **QTM camera system** (specific command depends on your QTM setup).

---

### 🔌 Terminal 2: Connect to the QCar

1. SSH into the QCar:
   ```bash
   ssh nvidia@192.168.1.126
   ```

2. When prompted, enter the password:
   ```text
   nvidia
   ```

3. Source the ROS 2 Humble environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

4. Navigate to the ROS 2 workspace and source it:
   ```bash
   cd ros2
   source install/setup.bash
   ```

5. Launch the QCar nodes:
   ```bash
   ros2 launch qcar2_nodes qcar2_launch.py
   ```

✅ Now the QCar's ROS 2 topics will be available from your development machine.

---

### 🚀 Terminal 3: Run Your Code Locally

1. In your ROS 2 development environment, make sure your workspace is sourced.
2. Run your ROS 2 node using:
   ```bash
   ros2 run <your_package_name> <your_script_name>.py
   ```

> Replace `<your_package_name>` and `<your_script_name>` with your actual package and script names.

---

## 📌 Notes
- Make sure all terminals are properly sourced with the correct ROS environment.
- Network communication requires both systems to be on the same subnet (e.g., 192.168.1.xxx).
- You may use tools like `rqt_graph` or `ros2 topic list` to debug connectivity.

---

Happy coding with your QCar! 🎉
