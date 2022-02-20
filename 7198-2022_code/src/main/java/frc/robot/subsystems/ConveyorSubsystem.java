// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase {
  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {

    //Inside Robot
      CANSparkMax conveyorMotor = new CANSparkMax(3, MotorType.kBrushless);

    //Sensors:
      //In Gate: Beam Break
        DigitalInput inGate_BB = new DigitalInput(1);
      //Mid Gate: Beam Break
        DigitalInput midGate_BB = new DigitalInput(2);
      //Shooter Gate: Beam Break
        DigitalInput shooter_BB = new DigitalInput(3);
      

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
