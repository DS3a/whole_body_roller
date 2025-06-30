/*
    The main control loop will be in this file,
    which will take joint data from the robot, and feed it to the dynamics model,
    along with any setpoints (task based, or joint based, TBD) 
    and update all the constraints that need to be updated 
    before solving a QP to determine the control decision variables.
*/