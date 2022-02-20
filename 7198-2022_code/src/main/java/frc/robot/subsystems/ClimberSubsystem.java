// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    //Tilt
    DoubleSolenoid tilt = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

    //Reach
    DoubleSolenoid reach = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

    //Static Hooks
    DoubleSolenoid hooks = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
