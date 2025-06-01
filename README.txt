//////////////////////////////
Library Name: fiveBar       //
Editor Name: Joel MacVicar  //
Last Modified: 3/15/25       //
//////////////////////////////

////////
Purpose: 

    A library which Mechancial Engineering students can use to design and implement 
controllers for a five bar robot. Because ME students have a wide variety of coding experieince,
having to fully develop their own software to study controls can be a challenging/discouraging 
task. This software library seeks to mitigage these struggles. 

////////////////
List of Features:

1. Kinematics (fiveBarKin.h) 
    a. Takes length parameters of each link when initializing. 
    b. Perform forward and inverse kinematics calculations for specified five bar robot 
       configuration. 
    c. Includes all for angle positions and velocities, and the x and y components of the end effector 
        position and velocity.
    d. FUTURE STEPS:
        - add accelerations of each parameter
2. Dynamics (fiveBarDyn.h)
    a. Takes length, mass, CoM, and moment of interia when initializing.
    b. Calculates the mass (M) and C matrices for the five bar equations of motion. 

3. Control methods (fiveBarCon.h)
    a. Takes an instance of the Kinematics and Dynamics class when initializing
    b. Uses calculations from the kinematics and dynamics classes to calculate motor torques for
        a control scheme.
    c. Current control methods include
        - canceling the non-linear dynamics (the C matrix)
        - computed torque for a specified tau.
    d. FUTURE STEPS:
        - There are countless controllers that are to be explored/researched for the five bar. 
          As these controllers are successfully implemented in Arudino IDE, converting them to a 
          function which can be added to this class will provide a useful library for students to
          reference/use in the future. 

////////////////////////
External Libraries Used:
Eigen 

//////////////////////////////
List of Compatible Environments:
1. Arduino IDE
2. VS Code

