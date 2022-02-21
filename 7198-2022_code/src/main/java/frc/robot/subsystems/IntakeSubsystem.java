// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.StopIntake;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  CANSparkMax rollerMotor;

  DoubleSolenoid intakePneumatic;


  public IntakeSubsystem() {

    //Roller Drive
      rollerMotor = new CANSparkMax(1, MotorType.kBrushless);

    //Pivot - Air
      intakePneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

          //Default command for the subsystem
          this.setDefaultCommand(new StopIntake(this));   
  }

  //Control motor speed
  public void set(final double s) {
    rollerMotor.set(s);
  }
  public void intake(){
    set(1.0);
  }
  public void eject(){
    set(-1.0);
  }

  public void stop(){
    rollerMotor.stopMotor();
  }


  //Control the pneumatic solenoid
  public void raiseIntake(){
    intakePneumatic.set(Value.kReverse);
  }

  public void lowerIntake(){
    intakePneumatic.set(Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
