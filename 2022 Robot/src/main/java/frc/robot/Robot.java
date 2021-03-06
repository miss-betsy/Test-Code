// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.IFollower;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Robot extends TimedRobot {
  
  //Autonomous Selection Variables
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final String k_taxiAuto = "Taxi Auto";
  private static final String k_ballShootingAuto = "Ball Auto";
  private static final String k_ScoreDriveTurnDriveCollect = "Fancy Auto";
  private static final String k_ScoreTurnDrive = "Long Loop";

  //Controller
  XboxController driverController = new XboxController(0);
  XboxController OPController = new XboxController(2);

  //Alliance Color Values
  String allianceColor = DriverStation.getAlliance().toString();
  String colorString;
  
  //Drive Train 
  //Drive: 4 Falcon motors with integrated motor controllers
  //Rio tells it that the system is the Rio, every other system will ignore
  WPI_TalonFX LeftFront = new WPI_TalonFX(1, "rio");
  WPI_TalonFX LeftRear = new WPI_TalonFX(3, "rio");
  WPI_TalonFX RightFront = new WPI_TalonFX(4, "rio");
  WPI_TalonFX RightRear = new WPI_TalonFX(2, "rio");
  double forward = 0.0;
  double turn = 0.0;

  //Drive Train Smoothing
  SlewRateLimiter throttleLimiter = new SlewRateLimiter(1.25);
  
  //Cargo Handling
  //Intake
  //Roller Drive: In/Out - Spark Max and Neo
  CANSparkMax rollerMotor = new CANSparkMax(5, MotorType.kBrushless);
  //Arm: Up/Down - Spark Max and Neo
  CANSparkMax intakeMotor = new CANSparkMax(6, MotorType.kBrushless );
  DigitalInput upperLimitSwitch = new DigitalInput(9);
  DigitalInput lowerLimitSwitch = new DigitalInput(8);

  //Sensors: Intgrated hall effect for motor speed

//DIO 8 & 9 for intake - 9 upper, 8 lower
  //Conveyor
  //Inside robot: Up/down - Spark Max and Neo
  CANSparkMax conveyorMotor = new CANSparkMax(7, MotorType.kBrushless);

  //Sensors: Integrated hall effect for motor speed +
  DigitalInput inGate_BB = new DigitalInput(3);   //In Gate: Beam Break
  DigitalInput midGate_BB = new DigitalInput(2);  //Mid Gate: Beam Break
  DigitalInput shooter_BB = new DigitalInput(1);  //Shooter Gate: Beam Break
  
  //Intake: Color Sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  //Adjust RBG valies based on scanning ball with shuffleboard
  Color kRedTarget = new Color(0.561, 0.232, 0.114);
  Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  Color detectedColor = m_colorSensor.getColor();
  ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    
  //Shooter: Falcon
  WPI_TalonFX shooterMotor = new WPI_TalonFX(8, "rio");

  //Conveyer states
  String conveyorState = "intake1"; 
  boolean intakeUpDownState = false;

  //Climber
  DoubleSolenoid clampPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3); //Clamps
  //DoubleSolenoid pneumaticreach = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5); 
  DoubleSolenoid pneumatictilt = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);  //Hooks - Tilt
  DoubleSolenoid pneumaticreach = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0); //Hooks - Reach
  Solenoid LEDLights = new Solenoid(PneumaticsModuleType.CTREPCM, 4); 

  //Time
  long matchTime;

  //Constants
  final double kZero = 0.0;
  final double kDeadband = 0.02;
  final double kIntakeSpeed = 0.6;
  final double kConveyorSpeed = 0.25;
  final double kShooterPercentSpin = 0.5;
  final double kShooterPercentShoot = 0.86;
  final double kShooterTarget = 18000.0;
  final double kShooterVelocity = 24000.0;
  final double kSpinupFeedforward = 0.5;
  final double kFireFeedforward = 0.80;
  final double kBlueConfidence = 0.9;
  final double kRedConfidence = 0.8;
  final double kYOLO = 0.95;
  final double kconveyorShoot = 0.30;
  final double kAutoSpeed = 0.5;
  final long kShooterDelay = 3000;
  final long kTaxiDelay = 10000;
  final double kTaxiDistance = 90000.0;
  final long kPulseLength = 100;
  final double kIntakeHold = 0.1;
  final double kUpDeploy = -0.25;
  final double kDownDeploy = 0.1;
  final double kAutoTestSpeed = 0.2;
  final double kAutoTestTurn = 0.33;
  final double kTravel2Turn = 50000.0;
  final double kAuto180 = 46000.00; //change to whatever we need to get to 180 degrees
  final double kTravel2Ball = 77500.0;
  final double kIntakeAutoSpeed = 0.8;


    @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Ball Auto", k_ballShootingAuto); //change back to Ball Auto 
    m_chooser.addOption("Ball Auto", k_ballShootingAuto);
    m_chooser.addOption("Fancy Auto", k_ScoreDriveTurnDriveCollect);
    m_chooser.addOption("Long Loop", k_ScoreTurnDrive);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    LeftFront.setSelectedSensorPosition(kZero);

    CameraServer.startAutomaticCapture();
    
    /* Ensure motor output is neutral during init */
    LeftFront.set(ControlMode.PercentOutput, kZero);
    LeftRear.set(ControlMode.PercentOutput, kZero);
    RightFront.set(ControlMode.PercentOutput, kZero);
    RightRear.set(ControlMode.PercentOutput, kZero);
    shooterMotor.set(ControlMode.PercentOutput, kZero);

    /* Factory Default all hardware to prevent unexpected behaviour */
    //LeftFront.configFactoryDefault();
    //LeftRear.configFactoryDefault();
    //RightFront.configFactoryDefault();
    //RightRear.configFactoryDefault();
    //shooterMotor.configFactoryDefault();

    /* Set Neutral mode */


    /* Configure output direction */
    LeftFront.setInverted(TalonFXInvertType.CounterClockwise);
    LeftRear.setInverted(TalonFXInvertType.CounterClockwise);
    RightFront.setInverted(TalonFXInvertType.Clockwise);
    RightRear.setInverted(TalonFXInvertType.Clockwise);
    shooterMotor.setInverted(TalonFXInvertType.CounterClockwise);
   }


  @Override
  public void robotPeriodic() {
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else {
      colorString = "Unknown";
    }
  
    detectedColor = m_colorSensor.getColor();
    match = m_colorMatcher.matchClosestColor(detectedColor);

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putBoolean("Intake Gate", inGate_BB.get());
    SmartDashboard.putBoolean("Mid Gate", midGate_BB.get());
    SmartDashboard.putBoolean("Shooter Gate", shooter_BB.get());
    SmartDashboard.putNumber("Shooter Encoder Value", shooterMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Drive Distance", LeftFront.getSelectedSensorPosition());
    SmartDashboard.putString("Conveyor State", conveyorState);
    SmartDashboard.putBoolean("Intake Up/Down", intakeUpDownState);
    SmartDashboard.putBoolean("Upper Limit Switch", upperLimitSwitch.get());
    SmartDashboard.putBoolean("Lower Limit Switch", lowerLimitSwitch.get());
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", k_ballShootingAuto);
    //m_autoSelected = SmartDashboard.getString("Fancy Auto", k_ScoreDriveTurnDriveCollect);
    //m_autoSelected = SmartDashboard.getString("Long Loop", k_ScoreTurnDrive);
    System.out.println("Auto selected: " + m_autoSelected);

    //tells the robot what alliance color we are
    allianceColor = DriverStation.getAlliance().toString();

    LeftFront.setSelectedSensorPosition(kZero);
    RightFront.setSelectedSensorPosition(kZero);
    LeftRear.setSelectedSensorPosition(kZero);
    RightRear.setSelectedSensorPosition(kZero);

    matchTime = System.currentTimeMillis();

    LeftFront.setNeutralMode(NeutralMode.Brake);
    LeftRear.setNeutralMode(NeutralMode.Brake);
    RightFront.setNeutralMode(NeutralMode.Brake);
    RightRear.setNeutralMode(NeutralMode.Brake);
    shooterMotor.setNeutralMode(NeutralMode.Brake);
    //intakeMotor.setNeutralMode(NeutralMode.Brake);
  }
  
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putString("Auto Mode", m_autoSelected);
    SmartDashboard.putNumber("Auto", System.currentTimeMillis() - matchTime);

    switch (m_autoSelected) {
      
      case k_taxiAuto:
        //taxi autonomous
        LeftFront.set(ControlMode.PercentOutput, -kAutoSpeed);
        LeftRear.set(ControlMode.PercentOutput, -kAutoSpeed);
        RightFront.set(ControlMode.PercentOutput, -kAutoSpeed);
        RightRear.set(ControlMode.PercentOutput, -kAutoSpeed);
        if (LeftFront.getSelectedSensorPosition() <= -kTaxiDistance) { //change 1.0 to proper value
          m_autoSelected = "autoEnd";
        }
      break;

      case "autoEnd":
        LeftFront.set(kZero);
        LeftRear.set(kZero);
        RightFront.set(kZero);
        RightRear.set(kZero);
        shooterMotor.set(ControlMode.PercentOutput, kZero);
        conveyorMotor.set(kZero);
      break;

      case k_ballShootingAuto: //remove quotes
        LeftFront.set(ControlMode.PercentOutput, kZero);
        LeftRear.set(ControlMode.PercentOutput, kZero);
        RightFront.set(ControlMode.PercentOutput, kZero);
        RightRear.set(ControlMode.PercentOutput, kZero);
        
        if ((System.currentTimeMillis() - matchTime) > kShooterDelay) { //shooter runs until empty. Change 3 to proper length
          shooterMotor.set(ControlMode.PercentOutput, kZero);
          conveyorMotor.set(kZero);
        } else {
          shooterMotor.set(ControlMode.PercentOutput, ShooterControl(kShooterVelocity, kFireFeedforward));
          if (shooterMotor.getSelectedSensorVelocity() >= kShooterTarget) {
            conveyorMotor.set(-kConveyorSpeed);
          }
        }
        if ((System.currentTimeMillis() - matchTime) > kTaxiDelay) { //shooter runs until empty. Change 3 to proper length
          m_autoSelected = "reverseTaxi2";
        }
      break;

      case "reverseTaxi2":
        LeftFront.set(ControlMode.PercentOutput, -kAutoSpeed);
        LeftRear.set(ControlMode.PercentOutput, -kAutoSpeed);
        RightFront.set(ControlMode.PercentOutput, -kAutoSpeed);
        RightRear.set(ControlMode.PercentOutput, -kAutoSpeed);
        shooterMotor.set(ControlMode.PercentOutput, kZero);
        conveyorMotor.set(kZero);
        if (LeftFront.getSelectedSensorPosition() <= -kTaxiDistance) { //change -5000.0 to proper value
          m_autoSelected = "autoEnd";
        }
      break;

      case k_ScoreTurnDrive:
          LeftFront.set(ControlMode.PercentOutput, kZero);
          LeftRear.set(ControlMode.PercentOutput, kZero);
          RightFront.set(ControlMode.PercentOutput, kZero);
          RightRear.set(ControlMode.PercentOutput, kZero);
          intakeMotor.set(kZero);
          
          if ((System.currentTimeMillis() - matchTime) > kShooterDelay) { //shooter runs until empty. Change 3 to proper length
            shooterMotor.set(ControlMode.PercentOutput, kZero);
            conveyorMotor.set(kZero);
          } else {
            shooterMotor.set(ControlMode.PercentOutput, ShooterControl(kShooterVelocity, kFireFeedforward));
            if (shooterMotor.getSelectedSensorVelocity() >= kShooterTarget) {
              conveyorMotor.set(-kConveyorSpeed);
            }
          }
          if ((System.currentTimeMillis() - matchTime) > kShooterDelay) { //shooter runs until empty. Change 3 to proper length
            m_autoSelected = "AutoTurn";
          }
      break;

      case "AutoTurn":
          LeftFront.set(ControlMode.PercentOutput, kAutoTestTurn);
          LeftRear.set(ControlMode.PercentOutput,kAutoTestTurn);
          RightFront.set(ControlMode.PercentOutput, -kAutoTestTurn);
          RightRear.set(ControlMode.PercentOutput, -kAutoTestTurn);
          if (RightFront.getSelectedSensorPosition() <= -kAuto180) {
            LeftFront.setSelectedSensorPosition(kZero); 
            m_autoSelected = "Drive2Ball";
          }
      break;

      case "Drive2Ball":
        LeftFront.set(ControlMode.PercentOutput, kAutoTestSpeed);
        LeftRear.set(ControlMode.PercentOutput, kAutoTestSpeed);
        RightFront.set(ControlMode.PercentOutput, kAutoTestSpeed);
        RightRear.set(ControlMode.PercentOutput, kAutoTestSpeed);
        rollerMotor.set(-kIntakeAutoSpeed);
        Timer.delay(0.2);
        if (inGate_BB.get()) {
         LeftFront.setSelectedSensorPosition(kZero); 
          m_autoSelected = "AutoTurn2";
        }
         
        case "AutoTurn2":
        LeftFront.set(ControlMode.PercentOutput, -kAutoTestTurn);
        LeftRear.set(ControlMode.PercentOutput,-kAutoTestTurn);
        RightFront.set(ControlMode.PercentOutput, kAutoTestTurn);
        RightRear.set(ControlMode.PercentOutput, kAutoTestTurn);
        rollerMotor.set(kZero);
        if (LeftFront.getSelectedSensorPosition() <= -kAuto180) {
          LeftFront.setSelectedSensorPosition(kZero); 
          m_autoSelected = "autoEnd";
        }
        break;

      /* case k_ScoreDriveTurnDriveCollect:
          LeftFront.set(ControlMode.PercentOutput, kZero);
          LeftRear.set(ControlMode.PercentOutput, kZero);
          RightFront.set(ControlMode.PercentOutput, kZero);
          RightRear.set(ControlMode.PercentOutput, kZero);
          intakeMotor.set(kZero);
          
          if ((System.currentTimeMillis() - matchTime) > kShooterDelay) { //shooter runs until empty. Change 3 to proper length
            shooterMotor.set(ControlMode.PercentOutput, kZero);
            conveyorMotor.set(kZero);
          } else {
            shooterMotor.set(ControlMode.PercentOutput, ShooterControl(kShooterVelocity, kFireFeedforward));
            if (shooterMotor.getSelectedSensorVelocity() >= kShooterTarget) {
              conveyorMotor.set(-kConveyorSpeed);
            }
          }
          if ((System.currentTimeMillis() - matchTime) > kShooterDelay) { //shooter runs until empty. Change 3 to proper length
            m_autoSelected = "reverse2turn";
          }
      break;

      case "reverse2turn":
        LeftFront.set(ControlMode.PercentOutput, -kAutoTestSpeed);
        LeftRear.set(ControlMode.PercentOutput,-kAutoTestSpeed);
        RightFront.set(ControlMode.PercentOutput, -kAutoTestSpeed);
        RightRear.set(ControlMode.PercentOutput, -kAutoTestSpeed);
        shooterMotor.set(ControlMode.PercentOutput, kZero);
        conveyorMotor.set(kZero);
        if (LeftFront.getSelectedSensorPosition() <= -kTravel2Turn) {
          m_autoSelected = "autoTurn";
        }
      break;

      case "autoTurn":
      LeftFront.set(ControlMode.PercentOutput, -kAutoTestSpeed);
      LeftRear.set(ControlMode.PercentOutput,-kAutoTestSpeed);
      RightFront.set(ControlMode.PercentOutput, kAutoTestSpeed);
      RightRear.set(ControlMode.PercentOutput, kAutoTestSpeed);
      if (RightFront.getSelectedSensorPosition() <= kAuto180) { 
        m_autoSelected = "forward2ball";
      }
      break;

      case "forward2ball":
      LeftFront.set(ControlMode.PercentOutput, kAutoTestSpeed);
      LeftRear.set(ControlMode.PercentOutput, kAutoTestSpeed);
      RightFront.set(ControlMode.PercentOutput, kAutoTestSpeed);
      RightRear.set(ControlMode.PercentOutput, kAutoTestSpeed);
      rollerMotor.set(kIntakeSpeed);

      if(inGate_BB.get()) {
        m_autoSelected = "autoTurn2";
      }
      break;

      case "autoTurn2":
      LeftFront.set(ControlMode.PercentOutput, -kAutoTestSpeed);
      LeftRear.set(ControlMode.PercentOutput,-kAutoTestSpeed);
      RightFront.set(ControlMode.PercentOutput, kAutoTestSpeed);
      RightRear.set(ControlMode.PercentOutput, kAutoTestSpeed);
      rollerMotor.set(kZero);
      if (LeftFront.getSelectedSensorPosition() <= kAuto180) { 
        m_autoSelected = "forward2Shoot";
      }
      break;

      case "forward2Shoot":
      LeftFront.set(ControlMode.PercentOutput, -kAutoTestSpeed);
      LeftRear.set(ControlMode.PercentOutput,-kAutoTestSpeed);
      RightFront.set(ControlMode.PercentOutput, -kAutoTestSpeed);
      RightRear.set(ControlMode.PercentOutput, -kAutoTestSpeed);
      shooterMotor.set(ControlMode.PercentOutput, kZero);
      conveyorMotor.set(kZero);
      if (LeftFront.getSelectedSensorPosition() <= 2*(-kTravel2Turn)) { 
        m_autoSelected = k_ballShootingAuto;
      }
    break; */

    }
  }

  @Override
  public void teleopInit() {
    LeftFront.setSelectedSensorPosition(kZero);

    allianceColor = DriverStation.getAlliance().toString();

    intakeUpDownState = false;

    conveyorState = "intake1";

    LeftFront.setNeutralMode(NeutralMode.Coast);
    LeftRear.setNeutralMode(NeutralMode.Coast);
    RightFront.setNeutralMode(NeutralMode.Coast);
    RightRear.setNeutralMode(NeutralMode.Coast);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    //intakeMotor.setNeutralMode(NeutralMode.Brake);
    
    /* Configure output direction */
    LeftFront.setInverted(TalonFXInvertType.CounterClockwise);
    LeftRear.setInverted(TalonFXInvertType.CounterClockwise);
    RightFront.setInverted(TalonFXInvertType.Clockwise);
    RightRear.setInverted(TalonFXInvertType.Clockwise);
    shooterMotor.setInverted(TalonFXInvertType.CounterClockwise);
  }

  @Override
  public void teleopPeriodic() {
    //Drivetrain Control
    /* Gamepad processing */                 
		//double left = -1.0 * Math.pow(driverController.getLeftY(), 3.0);
		//double right = -1.0 * Math.pow(driverController.getRightY(), 3.0);	
		//left = Deadband(left);
		//right = Deadband(right);

    //if (driverController.getLeftBumperPressed()) {
    //    forward = -1.0 * Math.pow(driverController.getLeftY(), 3.0);
    //    turn = 0.5 * driverController.getRightX();
    //    forward = Deadband(forward);
    //    turn = Deadband(turn); 
    //} else {
    //    forward = -.4 * driverController.getLeftY();
    //    turn = .2 * driverController.getRightX();
    //    forward = Deadband(forward);
    //    turn = Deadband(turn);
    //}
/*
      if (driverController.getLeftTriggerAxis() > 0.5) {
        forward = -1.0 * Math.pow(driverController.getLeftY(), 3.0);
        turn = 0.7 *driverController.getRightX();
      } else if (driverController.getLeftTriggerAxis() <0.5 && driverController.getLeftTriggerAxis() > 0.1) {
        forward = -0.7 * Math.pow(driverController.getLeftY(), 3.0);
        turn = 0.55 * driverController.getRightX();
      } else {
        forward = -0.5 * Math.pow(driverController.getLeftY(), 3.0);
        turn = 0.3 * driverController.getRightX();
      }
*/
      boolean isQuickTurn = Math.abs(forward) < 0.1;
      forward = -1.0 * Math.pow(driverController.getLeftY(), 3.0);
      turn = 0.6 * driverController.getRightX();

      forward = Deadband(forward);
      turn = Deadband(turn);

      double limitedForward = throttleLimiter.calculate(forward);

      
      WheelSpeeds wheelSpeed = DifferentialDrive.curvatureDriveIK(limitedForward, turn, isQuickTurn);

      LeftFront.set(ControlMode.PercentOutput, wheelSpeed.left);
      LeftRear.set(ControlMode.PercentOutput, wheelSpeed.left);
      RightFront.set(ControlMode.PercentOutput, wheelSpeed.right);
      RightRear.set(ControlMode.PercentOutput, wheelSpeed.right);


		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
    /*
		LeftFront.set(ControlMode.PercentOutput, limitedForward, DemandType.ArbitraryFeedForward, +turn);
		LeftRear.set(ControlMode.PercentOutput, limitedForward, DemandType.ArbitraryFeedForward,  +turn);
    RightFront.set(ControlMode.PercentOutput, limitedForward, DemandType.ArbitraryFeedForward,  -turn);
		RightRear.set(ControlMode.PercentOutput, limitedForward, DemandType.ArbitraryFeedForward,  -turn);
    */

    //Climber Control - change Pneumatictest to poneumaticreach
    if (OPController.getPOV() == 0) {
      pneumaticreach.set(Value.kForward); //down on dpad extends hooks (drop)
    }
    if (OPController.getPOV() == 180) {
      pneumaticreach.set(Value.kReverse); //up on dpad retracts hooks (pullup)
    }
    if (OPController.getPOV() == 90) {
      pneumatictilt.set(Value.kForward); // right leans hooks back
    }                              
    if (OPController.getPOV() == 270) {
      pneumatictilt.set(Value.kReverse); //left brings hooks back upright
    }
    if (OPController.getRawButtonPressed(7) && OPController.getRawButtonPressed(8)) {
      clampPneumatic.set(Value.kForward);  //back and start open clamps
    }
    if (OPController.getYButton()) {
      clampPneumatic.set(Value.kReverse); //Y closes clamps remove
    }
    
    /*
    if (controller.getRawButton(3)) { // X button shoot
      shooterMotor.set(ControlMode.PercentOutput, 0.9);
      SmartDashboard.putBoolean("Shooter Activated", true);
    } else {
      shooterMotor.set(ControlMode.PercentOutput, kZero);
      SmartDashboard.putBoolean("Shooter Activated", false);
    }
      //if (controller.getLeftBumperPressed() && controller.getRawButtonPressed(2)) { //left bumper + back barf
      // rollerMotor.set(1.0);
      //} 
      //if (controller.getLeftBumperReleased() && controller.getRawButtonReleased(2)) { //let go of left bumper + back barf stop
      //  rollerMotor.set(kZero);
      //}
    if (controller.getLeftBumperPressed()) { //left bumper intake
      rollerMotor.set(-1.0);
      centerMotor.set(-0.5);
    }
    if (controller.getLeftBumperReleased()) { //let go of left bumper stop roller motor
      rollerMotor.set(kZero);
      centerMotor.set(kZero);
    }
      // if (controller.getLeftTriggerAxis(0.5)) {//left trigger runs intake in reverse
      //  rollerMotor.set(1.0);
      //}
      //if (controller.getLeftTriggerAxis(0) {//let go of trigger to stop
      //  rollerMotor.set(kZero);
      //}
      //if (controller.getRightBumperPressed() && controller.getRawButtonPressed(2)) {  // right bumper + back conveyor out
      //  conveyorMotor.set(-kConveyorSpeed);
      //}
      //if (controller.getRightBumperReleased() && controller.getRawButtonReleased(2)) { //let go of right bumper + back conveyor out off
      //  conveyorMotor.set(kZero);
      //}
    if (controller.getRightBumperPressed()) {  // right bumper conveyor on
      conveyorMotor.set(-kConveyorSpeed);
    }
    if (controller.getRightBumperReleased()) { //let go of right bumper conveyor off
      conveyorMotor.set(kZero);
    }

    //if (controller.getRightTriggerAxis()) {
     // conveyorMotor.set(1.0);
    //}
    //if (controller.getRightTriggerAxis()) {
     // conveyorMotor.set(kZero)
    //}
    */
  
    /*
    //Conveyor Control w/ Color Sensor
    switch (conveyorState) {
      case "intake1": //what will execute while conveyor is empty
        //Intake control
        //If the left bumper button on the controller is pressed the intake is down(forward), 
        //Else it isn't pressed the intake is up(reverse)
        shooterMotor.set(kZero);
        conveyorMotor.set(kZero);

        if (controller.getLeftBumper()){
          if (controller.getAButton()){   //If they hit 'A' while the left bumper is pressed, it will eject, otherwise it will intake
            rollerMotor.set(kIntakeSpeed);
            centerMotor.set(kIntakeSpeed);
          } else {
            rollerMotor.set(-kIntakeSpeed);
            centerMotor.set(-kIntakeSpeed);
          }
        } else {  //Else stop all of the motors
          rollerMotor.set(kZero);
          centerMotor.set(kZero);
        }

        //if the beam break is broken, then the state moves to Stage2Color to run conveyor
        if (inGate_BB.get()) {
          conveyorState = "judgeColor1";
        }
      break;
      
      case "judgeColor1": 
        //stop conveyor
        conveyorMotor.set(kZero);

        //check the color sensor - must match the alliance color
        if (allianceColor == colorString) {          
          if ((colorString == "blue" && match.confidence >= kBlueConfidence) || (colorString == "red" && match.confidence >= kRedConfidence)){
            conveyorState = "move2Mid";
          }
        } else {
          conveyorState = "Ejectball1";
        }
      break;
      
      case "move2Mid": 
        //Run conveyor  
        conveyorMotor.set(-kConveyorSpeed);

        if (midGate_BB.get()) {
          conveyorState = "Intake2";
        } 
      break;
      
      case "Intake2": 
        //Intake control
        //If the left bumper button on the controller is pressed the intake is down(forward), 
        //else it isn't pressed the intake is up(reverse)
        shooterMotor.set(kZero);
        conveyorMotor.set(kZero);

        if (controller.getLeftBumper()){ //if they hit 'A' while the left bumper is pressed, it will eject, otherwise it will intake
          if (controller.getAButton()){
            rollerMotor.set(kIntakeSpeed);
            centerMotor.set(kIntakeSpeed);
          } else {
            rollerMotor.set(-kIntakeSpeed);
            centerMotor.set(-kIntakeSpeed);
          }
        } else {
          rollerMotor.set(kZero);
          centerMotor.set(kZero);
        }

        if (inGate_BB.get()) {
          conveyorState = "judge2color2";
        }

        if(controller.getXButton()) {
          conveyorState = "move2shooter";
        }
      break;

      case "judge2color2": 
        //stop conveyor
        conveyorMotor.set(kZero);

        //check the color sensor - must match the alliance color
        if (allianceColor == colorString) {
          if ((colorString == "blue" && match.confidence >= kBlueConfidence) || (colorString == "red" && match.confidence >= kRedConfidence)){
            conveyorState = "move2shooter";
          }
        } else {
          conveyorState = "Ejectball2";
        }            
      break;
      
      case "move2shooter":
        conveyorMotor.set(-kConveyorSpeed);
        if (shooter_BB.get()) {
          conveyorState = "spinup";
        }
      break;
      
      case "spinup": 
        rollerMotor.set(kZero);
        centerMotor.set(kZero);
        conveyorMotor.set(kZero);
        shooterMotor.set(ControlMode.PercentOutput, kShooterSpinup);
        
        if (controller.getXButton()) { //if the 'X' button is pushed, fire the balls
          conveyorState = "FireFireFire";
        }
      break;

      case "FireFireFire":
        shooterMotor.set(ControlMode.PercentOutput, kShooterVelocity);
        
        if (shooterMotor.getSelectedSensorVelocity() >= kShooterTarget) {
          conveyorMotor.set(-kConveyorSpeed);
          
          if (controller.getXButton() && !inGate_BB.get() && !midGate_BB.get() && !shooter_BB.get()) { //shooter runs when x is pushed and continues until x is pushed again. Do not hit x again until conveyor is empty
            conveyorState = "intake1";
          }
        }
      break;

      case "Ejectball1": 
        conveyorMotor.set(kConveyorSpeed);
        centerMotor.set(kIntakeSpeed);
        rollerMotor.set(kIntakeSpeed);

        if (!inGate_BB.get() && !midGate_BB.get() && !shooter_BB.get()) { //shooter runs when x is pushed and continues until x is pushed again. Do not hit x again until conveyor is empty
          conveyorState = "intake1";
        }
      break;

      case "Ejectball2a":
        conveyorMotor.set(kConveyorSpeed);
        centerMotor.set(kIntakeSpeed);
        rollerMotor.set(kIntakeSpeed);

        if (inGate_BB.get()) {
          conveyorState = "Ejectball2b";
        }
      break;

      case "EjectBall2b":
        conveyorMotor.set(kConveyorSpeed);
        centerMotor.set(kIntakeSpeed);
        rollerMotor.set(kIntakeSpeed);

        if (!inGate_BB.get()) {
          conveyorState = "intake2";
        }
      break;

      case "emergencyClear":
        conveyorMotor.set(kConveyorSpeed);
        centerMotor.set(kIntakeSpeed);
        rollerMotor.set(kIntakeSpeed);
       shooterMotor.set(ControlMode.PercentOutput, kZero);

        if (!inGate_BB.get() && !midGate_BB.get() && !shooter_BB.get()) { //shooter runs when x is pushed and continues until x is pushed again. Do not hit x again until conveyor is empty
          conveyorState = "intake1";
        }
      break;
    }
      
    //barf button
    if (controller.getLeftBumper() && controller.getRightBumper()) {
      conveyorState = "emergencyClear";
    }
    */

    //Conveyor Control w/ Color Sensor
    switch (conveyorState) {
      case "intake1": //what will execute while conveyor is empty
        //Intake control
        //If the left bumper button on the controller is pressed the intake is down(forward), 
        //Else it isn't pressed the intake is up(reverse)
        shooterMotor.set(ControlMode.PercentOutput, kZero);
        conveyorMotor.set(kZero);
        LEDLights.set(false);

        if (driverController.getLeftBumper()){
          if (driverController.getAButton()){   //If they hit 'A' while the left bumper is pressed, it will eject, otherwise it will intake
            rollerMotor.set(kIntakeSpeed);
          } else {
            rollerMotor.set(-1.0 * kIntakeSpeed);
          }
        } else {  //Else stop all of the motors
          rollerMotor.set(kZero);
        }

        //if the beam break is broken, then the state moves to Stage2Color to run conveyor
        if (inGate_BB.get()) {
          conveyorState = "move2Mid";
        }
      break;
      
      case "move2Mid": 
        //Run conveyor  
        conveyorMotor.set(-1.0 * kConveyorSpeed);

        if (midGate_BB.get()) {
          conveyorState = "Intake2";
        } 
      break;
      
      case "Intake2": 
        //Intake control
        //If the left bumper button on the controller is pressed the intake is down(forward), 
        //else it isn't pressed the intake is up(reverse)
        shooterMotor.set(kZero);
        conveyorMotor.set(kZero);

        if (driverController.getLeftBumper()){ //if they hit 'A' while the left bumper is pressed, it will eject, otherwise it will intake
          if (driverController.getAButton()){
            rollerMotor.set(kIntakeSpeed);
          } else {
            rollerMotor.set(-1.0 * kIntakeSpeed);
          }
        } else {
          rollerMotor.set(kZero);
        }

        if (inGate_BB.get()) {
          conveyorState = "move2shooter";
        }

        if(driverController.getXButton()) {
          conveyorState = "move2shooter";
        }
      break;
      
      case "move2shooter":
        conveyorMotor.set(-1.0 * kConveyorSpeed);
        if (shooter_BB.get()) {
          conveyorState = "spinup";
        }
      break;
      
      
      case "spinup": 
        rollerMotor.set(kZero);
        conveyorMotor.set(kZero);
        shooterMotor.set(ControlMode.PercentOutput, kShooterPercentSpin);
        LEDLights.set(true);
        
        if (driverController.getXButton()) { //if the 'X' button is pushed, fire the balls
          conveyorState = "FireFireFire";
        }
      break;

      case "FireFireFire":
        shooterMotor.set(ControlMode.PercentOutput, ShooterControl(kShooterVelocity, kFireFeedforward));
        
        if (shooterMotor.getSelectedSensorVelocity() >= kShooterTarget) {
          conveyorMotor.set(-kconveyorShoot); 
        }

        if (!driverController.getXButton() && !inGate_BB.get() && !midGate_BB.get() && !shooter_BB.get()) { //shooter runs when x is pushed and continues until x is pushed again. Do not hit x again until conveyor is empty
          conveyorState = "intake1";
        }
      break;

      case "emergencyClear":
        conveyorMotor.set(kConveyorSpeed);
        rollerMotor.set(kIntakeSpeed);
        shooterMotor.set(ControlMode.PercentOutput, kZero);

        if (!inGate_BB.get() && !midGate_BB.get() && !shooter_BB.get()) { //shooter runs when x is pushed and continues until x is pushed again. Do not hit x again until conveyor is empty
          conveyorState = "intake1";
        }
      break;
    }
      
    //barf button
    if (OPController.getLeftBumper() && OPController.getRightBumper()) {
      conveyorState = "emergencyClear";
    }
    if (driverController.getLeftBumper() && driverController.getRightBumper()) {
      conveyorState = "emergencyClear";
    }

    if (OPController.getAButton()) {
      rollerMotor.set(-kIntakeSpeed);
    }
  

    //intake control
    if (driverController.getBButton() && intakeUpDownState) {
      intakeUpDownState = false;
      intakeMotor.set(kDownDeploy);
      Timer.delay(0.9);
    }
    
    if (driverController.getBButton() && !intakeUpDownState) {
      intakeUpDownState = true;
      Timer.delay(0.5);
    }

    if (intakeUpDownState) {
      if (upperLimitSwitch.get()) {
        intakeMotor.set(kUpDeploy);
      } else {
      intakeMotor.set(kZero);
      }
    } else {
      intakeMotor.set(kZero);
    }
    /*if (intakeUpDownState) {
      if (!upperLimitSwitch.get()) {
        intakeMotor.set(kDownDeploy);
      } else {
        intakeMotor.set(kZero);
      }

      intakeMotor.set(kDownDeploy);
    } else {
      if (!lowerLimitSwitch.get()) {
        intakeMotor.set(-1.0 * kUpDeploy);
      } else {
        intakeMotor.set(0.0);
      } 
    } */ 
  } 

  double Deadband(double value) {
    if (value >= kDeadband) { //Upper deadband
      return value;
    }
    else if (value <= (-1.0 * kDeadband)) { //Lower deadband
      return value;
    }
    else {  //Outside deadband 
      return 0.0;
    }
  }

  double ShooterControl(double target, double feedforward){
    double output = (0.55 * ((1.0 - (shooterMotor.getSelectedSensorVelocity() / target)))) + feedforward;
    return output; //change 0.7 to some other value
  }
}

//omit lower limit switch
//pulse down to deploy
// up until limit switch