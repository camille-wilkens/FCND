# Project Solution Readme #

### Scenario 1: Hover in same spot ###

- Tuned `Mass` to 0.5 in `QuadControlParams.txt` to get the vehicle to hover in same place

<p align="center">
<img src="animations/scenerio1.gif" width="500"/>
</p>

Pass:

<p align="center">
<img src="img/scenario1.PNG" width="500"/>
</p>



### Scenario 2: Body rate and roll/pitch control ###


 1. Implemented body rate control

 - implemented the code in the function `GenerateMotorCommands()`
 
        float len = L / (2.f * sqrtf(2.f));   
        float p_bar = momentCmd.x / len; // x axis    
        float q_bar = momentCmd.y / len; // y axis 
        float r_bar = -momentCmd.z / kappa; // z axis  
        float c_bar = collThrustCmd;  

       `  // 3D DRONE-FULL-NOTEBOOK (Lesson 4) - Set Propeller Angular Velocities`
       cmd.desiredThrustsN[0] = (c_bar + p_bar + q_bar + r_bar) / 4.f;  // Front Left 
       cmd.desiredThrustsN[1] = (c_bar - p_bar + q_bar - r_bar) / 4.f; // Front Right  
       cmd.desiredThrustsN[2] = (c_bar + p_bar - r_bar - q_bar) / 4.f; //Rear left 
       cmd.desiredThrustsN[3] = (c_bar - p_bar - q_bar + r_bar) / 4.f; //Rear Right  

 - implemented the code in the function `BodyRateControl()`

       V3F I;  
       I.x = Ixx;     
       I.y = Iyy;  
       I.z = Izz;  

       momentCmd = I * kpPQR * (pqrCmd - pqr); 

 - Tuned `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot
 
       kpPQR =43,43, 15

2. Implemented roll / pitch control

 - implemented the code in the function `RollPitchControl()`
 
       if (collThrustCmd > 0) {
            float acc = -collThrustCmd / mass;
            float b_x_a = R(0, 2);
            float b_x_c = accelCmd.x / acc;
            float b_x_e = b_x_c - b_x_a;
            float b_x_p_term = kpBank * b_x_e; 
            float b_y_a = R(1, 2);
            float b_y_c = accelCmd.y / acc;
            float b_y_e = b_y_c - b_y_a;
            float b_y_p_term = kpBank * b_y_e; 

            pqrCmd.x = (R(1, 0) * b_x_p_term - R(0, 0) * b_y_p_term) / R(2, 2); 
            pqrCmd.y = (R(1, 1) * b_x_p_term - R(0, 1) * b_y_p_term) / R(2, 2); 
           } 
           else { 
              pqrCmd.x = 0.0; 
              pqrCmd.y = 0.0; 
           } `
          pqrCmd.z = 0.0;  

 - Tuned `kpBank` in `QuadControlParams.txt` to minimize settling time but avoid too much overshoot
 
       kpBank = 8 

<p align="center">
<img src="animations/scenerio2.gif" width="500"/>
</p>

Pass:

<p align="center">
<img src="img/scenario2.PNG" width="500"/>
</p>



### Scenario 3: Position/velocity and yaw angle control ###

1. Implemented the code in the function `LateralPositionControl()`

       if (velCmd.mag() > maxSpeedXY) {
             velCmd = (velCmd.norm() * maxSpeedXY);
         }

         else { 
             velCmd = velCmd; 
         }



         accelCmd = (kpPosXY * (posCmd - pos) + kpVelXY * (velCmd - vel) + accelCmd);

         if (accelCmd.mag() > maxAccelXY) {
             accelCmd = (accelCmd.norm() * maxAccelXY);

         }
      
2.  Implemented the code in the function `AltitudeControl()`

            float z_err = posZCmd - posZ;
            float z_err_dot = velZCmd - velZ;

            float p_term = kpPosZ * z_err;
            float d_term = kpVelZ * z_err_dot + velZ;

            //integrator
            integratedAltitudeError += z_err * dt;

            float u_1_bar = p_term + d_term + (integratedAltitudeError * KiPosZ) + accelZCmd;

            float y = R(2, 2);
            float c = (u_1_bar - CONST_GRAVITY) /y;

            thrust = -mass * CONSTRAIN(c, -maxAscentRate / dt, maxAscentRate / dt);
      
3.  Tuned parameters `kpPosZ` and `kiPosZ` in `QuadControlParams.txt`

          kpPosZ = 25
          KiPosZ = 40

4.  Tuned parameters `kpVelXY` and `kpVelZ`in `QuadControlParams.txt`

          kpVelXY = 12.0
          kpVelZ = 10.0

5.  Implemented the code in the function `YawControl()`

        yawRateCmd = fmodf(yawCmd, 2 * F_PI);

        float yaw_Error = yawRateCmd - yaw;

        if (yaw_Error > F_PI) {
            yaw_Error -= 2.0 * F_PI;
        }
        if (yaw_Error < -F_PI) {
            yaw_Error += 2.0 * F_PI;

        }

        yawRateCmd = yaw_Error * kpYaw;

6.  Tuned parameters `kpYaw` and the 3rd (z) component of `kpPQR` in `QuadControlParams.txt`

        kpYaw = 2
        kpPQR =43,43, 15

<p align="center">
<img src="animations/scenerio3.gif" width="500"/>
</p>

Pass:
<p align="center">
<img src="img/scenario3.PNG" width="500"/>
</p>


### Scenerio 4: Non-idealities and robustness  ###

In this part, we will explore some of the non-idealities and robustness of a controller. 

1. Tweaked the controller parameters to work for all 3 (tip: relax the controller) - in `QuadControlParams.txt`

       # Physical properties
       Mass = 0.5
       L = 0.17
       Ixx = 0.0023
       Iyy = 0.0023
       Izz = 0.0046
       kappa = 0.016
       minMotorThrust = .1
       maxMotorThrust = 4.5

       # Position control gains
       kpPosXY = 30
       kpPosZ = 25
       KiPosZ = 40

       # Velocity control gains
       kpVelXY = 12.0
       kpVelZ = 10.0

       # Angle control gains
       kpBank = 8
       kpYaw = 2

       # Angle rate gains
       kpPQR =43,43, 15

       # limits
       maxAscentRate = 5
       maxDescentRate = 2
       maxSpeedXY = 5
       maxHorizAccel = 12
       maxTiltAngle = .7

2. Edited `AltitudeControl()` to add basic integral control to help with the different-mass vehicle.

          float z_err = posZCmd - posZ;
          float z_err_dot = velZCmd - velZ;

          float p_term = kpPosZ * z_err;
          float d_term = kpVelZ * z_err_dot + velZ;

          //integrator
          integratedAltitudeError += z_err * dt;

          float u_1_bar = p_term + d_term + (integratedAltitudeError  * KiPosZ) + accelZCmd;

          float y = R(2, 2);
          float c = (u_1_bar - CONST_GRAVITY) /y;

          thrust = -mass * CONSTRAIN(c, -maxAscentRate / dt, maxAscentRate / dt);



<p align="center">
<img src="animations/scenerio4.gif" width="500"/>
</p>

Pass:
<p align="center">
<img src="img/scenario4.PNG" width="500"/>
</p>

### Scenerio 5: Tracking trajectories ###
<p align="center">
<img src="animations/scenerio5.gif" width="500"/>
</p>

Pass:
<p align="center">
<img src="img/scenario5.PNG" width="500"/>
</p>




### Performance Metrics ###

The specific performance metrics are as follows:

 - scenario 2
   - roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop)
   - roll rate should less than 2.5 radian/sec for 0.75 seconds

 - scenario 3
   - X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds
   - Quad2 yaw should be within 0.1 of the target for at least 1 second


 - scenario 4
   - position error for all 3 quads should be less than 0.1 meters for at least 1.5 seconds

 - scenario 5
   - position error of the quad should be less than 0.25 meters for at least 3 seconds


