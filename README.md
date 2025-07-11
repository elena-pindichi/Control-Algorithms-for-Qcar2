# Control Implementation of Safe and Secure Self-Driving Car - Quanser
## Model Predictive Control implementations:
### Nonlinear MPC:
- In the script 'NL_MPC_qcar.m' is the implementation of Nonlinear MPC with reference tracking.
- The reference created are either a line, a square, a circle or a B-Spline precomputed.
- It returns also the RMSE and the computation time for solving the optimization problem.

### Feedback Linearized MPC:
- In the script 'FL_MPC_qcar.m' is the implementation of Feedback Linearized MPC with reference tracking.
- In the folder 'tools' you can extract the functions that define the flat output, the mapping matrix for the control input and the matrix for the nonlinear constraints.
- It returns also the RMSE and the computation time for solving the optimization problem.

### Simulink for the real robot:
- In the folder 'quarc-simulink' there is the integration of the control algorithms with reference tracking set on the actual robot.
- The solver used for this integration was: 'fmincon'.
- It is still undergoing work to integrate Casadi through Quarc.

### ROS package folder:
- It has the functions added to a custom package in ROS to run the MPC algorithms on the real robot.

### Casadi in Simulink folder:
- It has the same implementation of the algorithms integrated with Casadi.