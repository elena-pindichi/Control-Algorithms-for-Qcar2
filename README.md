# 🤖 Control Implementation of a Safe and Secure Self-Driving Car – Quanser

This repository features several Model Predictive Control (MPC) strategies for autonomous trajectory tracking and real-time control of the QCar robot platform.

---

## 📊 Model Predictive Control (MPC) Implementations

### 🔁 Nonlinear MPC (NMPC)

- Implemented in: `NL_MPC_qcar.m`
- Supports multiple trajectory types:
  - Line
  - Square
  - Circle
  - Precomputed B-Spline
- Outputs:
  - Root Mean Square Error (RMSE)
  - Computation time for solving the optimization

---

### ➕ Feedback Linearized MPC (FLMPC)

- Implemented in: `FL_MPC_qcar.m`
- Additional helper functions are located in the `tools/` directory:
  - Flat output definition
  - Control input mapping matrix
  - Nonlinear constraints matrix
- Outputs:
  - RMSE
  - Computation time

---

### 🧩 Simulink Integration for Real-Time QCar Control

- Located in the `quarc-simulink/` folder
- Contains integration of the NMPC and FLMPC controllers for the real QCar platform
- Optimization is solved using MATLAB’s `fmincon`
- 🚧 **Note**: Work is in progress to integrate CasADi through QUARC

---

### 🐢 ROS Package Folder

- Contains ROS-compatible scripts for deploying the MPC controllers on the QCar robot
- Designed as a custom ROS 2 package
- Ensures real-time compatibility with ROS 2 nodes and topics

---

### 🧮 CasADi in Simulink Folder

- Includes CasADi-based implementations of both NMPC and FLMPC
- Provides symbolic optimization and code generation for embedded execution within Simulink

---

## 📁 Folder Summary

| Folder                | Description                                              |
|-----------------------|----------------------------------------------------------|
| `quarc-simulink/`     | Simulink + real robot integration using `fmincon`        |
| `tools/`              | Helper functions for flat outputs and constraints        |
| `casadi-in-simulink/` | CasADi-enhanced MPC implementation inside Simulink       |
| `ros2-package/`       | ROS 2 node integration for MPC-based control             |

---

## 🚀 Ready to Explore

Use these modules to simulate, validate, and deploy your autonomous driving strategies using Quanser's QCar platform.

> Contributions and experiments welcome! Happy testing 🧪🚘
