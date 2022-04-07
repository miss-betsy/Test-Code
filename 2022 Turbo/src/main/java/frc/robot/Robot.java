// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
//import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Controller
  XboxController driverController = new XboxController(0);

  //Drive Train
  Spark leftDrive = new Spark(0);
  Spark rightDrive = new Spark(1);
  //double forward = 0.0;
  //double turn = 0.0;

  DifferentialDrive drive = new DifferentialDrive(leftDrive, rightDrive);

  //Intake
  //in/out
  CANSparkMax intakeMotor = new CANSparkMax(3, MotorType.kBrushless);
  //up/down
  DoubleSolenoid intakePneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  //Conveyor
  VictorSPX conveyorMotorUpper = new VictorSPX(4);
  VictorSPX conveyorMotorLower = new VictorSPX(5);
  
  //Shooter
  WPI_TalonFX shooterMotor = new WPI_TalonFX(6);

  //climber
  VictorSPX climberMotorFront = new VictorSPX(7);
  VictorSPX climberMotorBack = new VictorSPX(8);

  //Constants
  final double kZero = 0.0;
  final double kConveyorSpeed = 0.7;
  final double kShooterPercent = 0.60;
  final double kClimberMove = 0.2;
  final double kintakePercent = 0.5;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    conveyorMotorLower.set(ControlMode.PercentOutput, kZero);
    conveyorMotorUpper.set(ControlMode.PercentOutput, kZero);
    climberMotorBack.set(ControlMode.PercentOutput, kZero);
    climberMotorBack.set(ControlMode.PercentOutput, kZero);
    shooterMotor.set(ControlMode.PercentOutput, kZero);
    rightDrive.setInverted(true);
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.setInverted(TalonFXInvertType.CounterClockwise);
    climberMotorBack.setNeutralMode(NeutralMode.Brake);
    climberMotorFront.setNeutralMode(NeutralMode.Brake);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
     //Drive Processing
    //forward = -1.0 * (driverController.getLeftY());
    //turn = .5 * (driverController.getRightX());
    drive.arcadeDrive((-1* driverController.getLeftY()), driverController.getRightX()); 

    //leftDrive.set(forward);
    //rightDrive.set(forward);

     if (driverController.getBButton()) {
       conveyorMotorLower.set(ControlMode.PercentOutput, kConveyorSpeed);
       conveyorMotorUpper.set(ControlMode.PercentOutput, kConveyorSpeed);
     } else {
       conveyorMotorLower.set(ControlMode.PercentOutput, kZero);
       conveyorMotorUpper.set(ControlMode.PercentOutput, kZero);
     }

     if (driverController.getLeftBumper()) {
       shooterMotor.set(ControlMode.PercentOutput, kShooterPercent);
     } else {
       shooterMotor.set(ControlMode.PercentOutput, kZero);
     }

     if (driverController.getAButton()) {
       intakeMotor.set(kintakePercent);
     } else {
       intakeMotor.set(kZero);
     }
 }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
