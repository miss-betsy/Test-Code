What I'm aiming for:

1 Cargo Handling

      a.  Intake
           i. Pivot (Double Acting Solenoid to Piston)
                 1. Extend (intake is out of the frame)
                 2. Retract (intake is stowed inside the frame)
          ii. Roller Drive (SparkMax on CAN to NEO motor)
                 1. In (Appropriate RPM Set Point, but as fast as possible)
                 2. Out (Appropriate RPM Set Point to keep balls out of the way during field traversal)
         iii. Sensors
                 1. Integrated hall effect for motor speed
      
  b. Centering
         i. Left and Right conveyor segments (SparkMax on CAN to NEO motor)
              1. Center (appropriate RPM set point)
              2. Eject (As fast as possible - RPM)
        ii. Sensors
              1. Integrated hall effect for motor speed
      
  c. Conveyor
        i. Conveyor segment from centering section to shooter (SparkMax on CAN to NEO motor)
              1. Up 
              2. Down
       ii. Sensors
              1. In Gate: Beam Break - Bottom of conveyor - ball radius inside up from lower conveyor roller
              2. Mid Gate: Beam Break - Middle of Conveyor - Ball diameter after In Gate
              3. Shooter Gate: Beam Break - ⅔ ball diameter ahead of shooter
              4. Color Sensor: Ball radius up stream of In Gate to determine whether to either:
                   a. Stage the ball to the mid gate
                   b. Run the conveyor up until mid gate is broken
                   c. Eject the ball
                   d. Run the conveyor down and Centering in Eject to spit the ball out to one side of the robot or the other.
              5. Integrated hall effect for motor speed
              
    d. Shooter
        i. 1. Run the shooter in the direction of projecting the ball out of the robot
            1. Idle: Maintain about 60% shooter speed so that we can get to speed faster when we are ready to shoot
            2. Shoot: Set the shooter to the appropriate RPM setpoint for the experimentally determined best surface speed

2. Hanging
    a. Arms
        i. Tilt (Double Acting Solenoid to Piston)
            1. Vertical (retracted)
            2. Tilted (extended)
        ii. Reach (Double Acting Solenoid to Piston)
            1. Reach (extend)
            2. Pull (retract)
     b. Static Hooks (Double Acting Solenoid to Piston)
        i. Clamp (extend)
            1. Release (retract)
            
3. Drive
