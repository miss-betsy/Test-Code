// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

  TalonFX motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft;

  public DrivetrainSubsystem() {
    motorFrontRight = new TalonFX(00);
    motorFrontLeft = new TalonFX(01);
    motorBackRight = new TalonFX(02);
    motorBackLeft = new TalonFX(03);

    motorFrontRight.setInverted(TalonFXInvertType.CounterClockwise);
    motorBackRight.setInverted(TalonFXInvertType.CounterClockwise);

    motorBackLeft.setInverted(TalonFXInvertType.CounterClockwise);
    motorFrontLeft.setInverted(TalonFXInvertType.CounterClockwise);

  }

  public void setPower(double leftPower, double rightPower){
    motorFrontRight.set(ControlMode.PercentOutput, rightPower);
    motorBackRight.set(ControlMode.PercentOutput, rightPower);

    motorFrontLeft.set(ControlMode.PercentOutput, leftPower);
    motorBackLeft.set(ControlMode.PercentOutput, leftPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
