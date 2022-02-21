// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Drive;

public class DrivetrainSubsystem extends SubsystemBase {

  WPI_TalonFX motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft;
  DifferentialDrive drive = new DifferentialDrive(motorFrontLeft, motorFrontRight);

  public DrivetrainSubsystem() {
    motorFrontRight = new WPI_TalonFX(00);
    motorFrontLeft = new WPI_TalonFX(01);
    motorBackRight = new WPI_TalonFX(02);
    motorBackLeft = new WPI_TalonFX(03);

    motorFrontRight.setInverted(TalonFXInvertType.CounterClockwise);
    motorBackRight.setInverted(TalonFXInvertType.CounterClockwise);

    motorBackLeft.setInverted(TalonFXInvertType.Clockwise);
    motorFrontLeft.setInverted(TalonFXInvertType.Clockwise);

    
  }
  
  public void arcadeDrive(double s, double r){
    drive.arcadeDrive(s, r);
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
