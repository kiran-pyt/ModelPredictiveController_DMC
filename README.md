# ModelPredictiveController_DMC



Dynamics Matrix Controller for Multi-Input-Multi-Output (MIMO) Systems
This MATLAB script implements a Dynamics Matrix Controller for Multi-Input-Multi-Output systems. The controller is designed to regulate the outputs of the system to desired reference values by manipulating the inputs while considering input rate constraints and input/output constraints.

Requirements
MATLAB software environment
Usage
Ensure MATLAB is installed on your system.
Open MATLAB and navigate to the directory containing the script (Dynamics_Matrix_Controller_MIMO.m).
Run the script by typing Dynamics_Matrix_Controller_MIMO in the MATLAB command window.
The script will execute the control algorithm and plot the system's response.
Description
Dynamics_Matrix_Controller_MIMO.m: This is the main MATLAB script implementing the Dynamics Matrix Controller.
The script defines the plant transfer functions (G11 and G22), controller parameters, constraints, and weights.
It computes the control inputs (du) using quadratic programming to minimize the predicted error between the output and reference trajectory.
The script iterates over time steps to simulate the system's response and stores the results.
Finally, it plots the output trajectories and control inputs.
Files
Dynamics_Matrix_Controller_MIMO.m: MATLAB script for the Dynamics Matrix Controller.
README.md: This README file providing information about the script.
Additional Notes
The script may require adjustments based on specific system dynamics and control requirements.
Ensure all necessary MATLAB toolboxes are installed for proper execution, especially the Optimization Toolbox for the quadprog function.
