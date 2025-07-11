# Control Implementation of Safe and Secure Self-Driving Car - Quanser
## Model Predictive Control implementations:
### Nonlinear MPC:
- In the script 'NL_MPC_qcar.m' is the implementation of Nonlinear MPC with reference tracking.
- The reference created are either a line, a square, a circle or a B-Spline precomputed.
- It returns also the RMSE and the time taken for solving the optimization problem.

### Feedback Linearized MPC:
- In the script 'FL_MPC_qcar.m' is the implementation of Feedback Linearized MPC with reference tracking.
- In the folder 'tools' you can extract the functions that define the flat output, the mapping matrix for the control input and the matrix for the nonlinear constraints.
- It returns also the RMSE and the time taken for solving the optimization problem.

### Simulink for the real robot:
- In the folder 'CarSimulink' there is the integration of the control algorithms with reference tracking set on the actual robot.
- The solver used for this integration was: 'fmincon'.

### Python integration folder:
- It has the control algorithms implemented in python for future usage in ROS or for other plottings.

### Introduce Casadi in Simulink folder:
- It has the same implementation of the algorithms integrated with Casadi.