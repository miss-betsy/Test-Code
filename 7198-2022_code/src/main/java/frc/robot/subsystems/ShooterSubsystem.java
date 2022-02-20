// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    //Shooter Motor
      TalonFX shooterMotor = new TalonFX(4);

    //Set Direction
    shooterMotor.setInverted(TalonFXInvertType.CounterClockwise);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
