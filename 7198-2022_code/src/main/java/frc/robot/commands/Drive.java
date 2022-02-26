// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  DrivetrainSubsystem m_deivetrain;
  public Drive(DrivetrainSubsystem drivetrain) {
    m_deivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //print statement vales of controllers
    System.out.println(RobotContainer.controller.getLeftY()+"  "+RobotContainer.controller.getRightX());

    //Send joystick commands to the drivetrain subsystem
    RobotContainer.drive.setPower(RobotContainer.controller.getLeftY(),
                                     RobotContainer.controller.getRightX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //test
}
