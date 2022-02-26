// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.StopIntake;
import frc.robot.subsystems.CenteringSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Controllers
  public static XboxController controller = new XboxController(0);

  //intake    
  //5 is right bumper
      JoystickButton intakeButton = new JoystickButton(controller, 5);


  // The robot's subsystems and commands are defined here...
  public static final CenteringSubsystem centering = new CenteringSubsystem();
  public static final ClimberSubsystem climber = new ClimberSubsystem();
  public static final ConveyorSubsystem conveyor = new ConveyorSubsystem();
  public static final DrivetrainSubsystem drive = new DrivetrainSubsystem();
  public static final IntakeSubsystem eat = new IntakeSubsystem();
  public static final ShooterSubsystem shooter = new ShooterSubsystem();


  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    // Configure the button bindings
    configureButtonBindings();

    //set defaults
    setAllDefaultCommands();
  }
  
  private void setAllDefaultCommands(){
    setDefaultCommand(drive, new Drive(drive));
    setDefaultCommand(eat, new StopIntake(eat));
  }


  private void setDefaultCommand(Subsystem subsystem, Command defaultCommand){
     CommandScheduler.getInstance().setDefaultCommand(subsystem, defaultCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //controller.rightBumper.whileHeld(new IntakeBalls());


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
  //}
}
