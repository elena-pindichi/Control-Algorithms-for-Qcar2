# QCar Simulink Deployment Guide

This guide provides instructions for configuring Simulink, generating code, and deploying it to a QCar or development machine.

---

## 🔧 Model Configuration Settings

Follow these steps after opening your Simulink model:

### 1. Solver Settings
- Go to **Model Configuration Settings** → **Solver**.
- Under **Solver selection**:
  - Set **Type** to: `Fixed-step`
  - Choose an appropriate **Solver** (e.g., `ode3`, `ode5`, etc.).
- Under **Solver details**:
  - Set **Fixed-step size** to the desired time step (e.g., `0.001`).

### 2. Code Generation: Target Selection
- Go to **Code Generation** → **Target selection**.
- Set **System target file** depending on your target:
  - **Development machine**: `quarc_win64.tlc` (Windows platform – good for real-time simulation and testing).
  - **QCar target**: `quarc_linux_nvidia.tlc`.

### 3. Code Generation: Network Configuration for QCar
If deploying to **QCar**, specify the target IP and port:

- Go to **Code Generation** → **Interface**.
- Under **MEX file arguments**, add:

  ```matlab
  '-w -d /tmp -uri %u', 'tcpip://IP_ADDRESS:PORT'

- For the car, I used:
  ```matlab
  '-w -d /tmp -uri %u', 'tcpip://192.168.1.126:17001'
