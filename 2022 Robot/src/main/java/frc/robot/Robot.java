// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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

  //Controller
  XboxController controller = new XboxController(0);

  //Alliance Color Values
  String allianceColor = DriverStation.getAlliance().toString();
  String colorString;

  //Shooter Timer For Autonomous
  Timer ShooterTimer;
  
  //Drive Train
  //Drive: 4 Falcon motors with integrated motor controllers
  //Rio tells it that the system is the Rio, every other system will ignore
  WPI_TalonFX LeftFront = new WPI_TalonFX(1, "rio");
  WPI_TalonFX LeftRear = new WPI_TalonFX(3, "rio");
  WPI_TalonFX RightFront = new WPI_TalonFX(4, "rio");
  WPI_TalonFX RightRear = new WPI_TalonFX(2, "rio");

  //Cargo Handling
  //Intake
  //Roller Drive: In/Out - Spark Max and Neo
  CANSparkMax rollerMotor = new CANSparkMax(5, MotorType.kBrushless);

  //Sensors: Intgrated hall effect for motor speed
  //Centering
  //Left and Right: Center/Eject - Spark Max and Neo
  CANSparkMax centerMotor = new CANSparkMax(6, MotorType.kBrushless);

  //Sensors: Integrated hall effect for motor speed
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

  //Climber
  DoubleSolenoid clampPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3); //Clamps
  DoubleSolenoid pneumaticreach = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5); //Hooks - Reach
  DoubleSolenoid pneumatictilt = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);  //Hooks - Tilt

  //Constants
  double kZero = 0.0;
  double kDeadband = 0.06;
  double kIntakeSpeed = 0.5;
  double kConveyorSpeed = 0.25;
  double kShooterPercentSpin = 0.5;
  double kShooterPercentShoot = 0.86;
  double kShooterSpinup = 10000.0;
  double kShooterTarget = 17000.0;
  double kShooterVelocity = 17500.0;
  double kBlueConfidence = 0.9;
  double kRedConfidence = 0.8;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Taxi Auto", k_taxiAuto);
    m_chooser.addOption("Ball Auto", k_ballShootingAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget); 
    m_colorMatcher.addColorMatch(kGreenTarget); 
    m_colorMatcher.addColorMatch(kYellowTarget); 
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
    SmartDashboard.putString("Conveyor State", conveyorState);
  }

  @Override
  public void autonomousInit() {
    ShooterTimer.reset();
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    //tells the robot what alliance color we are
    allianceColor = DriverStation.getAlliance().toString();
  }
  
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      
      case k_taxiAuto:
       // taxi autonomous
         LeftFront.set(1.0); //depending on what direction the robot is facing negative or positive
         LeftRear.set(1.0);
         RightFront.set(1.0);
         RightRear.set(1.0);
         if (LeftFront.getSelectedSensorPosition() >= 1.0) { //change 1.0 to proper value
          m_autoSelected = "autoEnd";
         }
        break;

        case "autoEnd":
        LeftFront.set(kZero);
        LeftRear.set(kZero);
        RightFront.set(kZero);
        RightRear.set(kZero);
        shooterMotor.set(ControlMode.Velocity, kZero);
        conveyorMotor.set(kZero);
        break;

      case k_ballShootingAuto:
         //drive to the goal
         LeftFront.set(ControlMode.Position, 2500.0); //change 2500.0 to proper value
         LeftRear.set(ControlMode.Position, 2500.0);
         RightFront.set(ControlMode.Position, 2500.0);
         RightRear.set(ControlMode.Position, 2500.0);
         shooterMotor.set(ControlMode.Velocity, kShooterSpinup); //change 1.0 to actual shooter velocity 100%
         if (LeftFront.getSelectedSensorPosition() >= 2500.0) { //change 2500.0 to proper value
          m_autoSelected = "FireFireFire";
         }
         //move through states to shoot
         //back out of taxi area
      break;

      case "FireFireFire":
      ShooterTimer.start();
      //if falcon is up to speed
      shooterMotor.set(ControlMode.Velocity, kShooterVelocity);
      shooterMotor.getSelectedSensorVelocity(); 
      if (shooterMotor.getSelectedSensorVelocity() >= kShooterTarget) {
        conveyorMotor.set(kConveyorSpeed);
        if (ShooterTimer.get() > 3.0) { //shooter runs until empty. Change 3 to proper length
          m_autoSelected = "reverseTaxi2";
        }
      }
      break;

      case "reverseTaxi2":
      LeftFront.set(ControlMode.Position, -5000.0); //change -5000.0 to proper value
      LeftRear.set(ControlMode.Position, -5000.0);
      RightFront.set(ControlMode.Position, -5000.0);
      RightRear.set(ControlMode.Position, -5000.0);
      shooterMotor.set(ControlMode.Velocity, kZero);
      conveyorMotor.set(kZero);
      if (LeftFront.getSelectedSensorPosition() <= -5000.0) { //change -5000.0 to proper value
       m_autoSelected = "autoEnd";
      }
      break;

    }
  }

  @Override
  public void teleopInit() {

    allianceColor = DriverStation.getAlliance().toString();

    conveyorState = "intake1";

    /* Ensure motor output is neutral during init */
    LeftFront.set(ControlMode.PercentOutput, 0);
    LeftRear.set(ControlMode.PercentOutput, 0);
    RightFront.set(ControlMode.PercentOutput, 0);
    RightRear.set(ControlMode.PercentOutput, 0);
    shooterMotor.set(ControlMode.PercentOutput, 0);

    /* Factory Default all hardware to prevent unexpected behaviour */
    LeftFront.configFactoryDefault();
    LeftRear.configFactoryDefault();
    RightFront.configFactoryDefault();
    RightRear.configFactoryDefault();
    shooterMotor.configFactoryDefault();

    /* Set Neutral mode */
    LeftFront.setNeutralMode(NeutralMode.Coast);
    LeftRear.setNeutralMode(NeutralMode.Brake);
    RightFront.setNeutralMode(NeutralMode.Coast);
    RightRear.setNeutralMode(NeutralMode.Brake);
    shooterMotor.setNeutralMode(NeutralMode.Coast);

    //shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    /* Configure output direction */
    LeftFront.setInverted(TalonFXInvertType.CounterClockwise);
    LeftRear.setInverted(TalonFXInvertType.CounterClockwise);
    RightFront.setInverted(TalonFXInvertType.Clockwise);
    RightRear.setInverted(TalonFXInvertType.Clockwise);
    shooterMotor.setInverted(TalonFXInvertType.CounterClockwise);

    /* Set Sensor Phase */
    //shooterMotor.setSensorPhase(true);
    //shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
  }

  @Override
  public void teleopPeriodic() {
    //Drivetrain Control
    /* Gamepad processing */                 
		double forward = -1.0 * Math.pow(controller.getLeftY(), 3.0);
		double turn = 0.5 * controller.getRightX();
    //double forward = -1.0 * controller.getLeftY();
		//double turn = controller.getRightX();		
		forward = Deadband(forward);
		turn = Deadband(turn);

		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
		LeftFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
		LeftRear.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
    RightFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		RightRear.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);

    //Climber Control
    if (controller.getPOV() == 0) {
      pneumaticreach.set(Value.kForward); //up on dpad extends hooks
    }
    if (controller.getPOV() == 180) {
      pneumaticreach.set(Value.kReverse); //down on dpad retracts hooks
    }
    if (controller.getPOV() == 90) {
      pneumatictilt.set(Value.kForward); // right leans hooks back
    }                              
    if (controller.getPOV() == 270) {
      pneumatictilt.set(Value.kReverse); //left brings hooks back upright
    }
    if (controller.getRawButtonPressed(7) && controller.getRawButtonPressed(8)) {
      clampPneumatic.set(Value.kForward);  //back and start open clamps
    }
    if (controller.getYButton()) {
      clampPneumatic.set(Value.kReverse); //Y closes clamps remove
    }
    
    /*
    if (controller.getRawButton(3)) { // X button shoot
      shooterMotor.set(ControlMode.PercentOutput, 0.9);
      //shooterMotor.set(ControlMode.Velocity, -kShooterVelocity);
      SmartDashboard.putBoolean("Shooter Activated", true);
    } else {
      shooterMotor.set(ControlMode.PercentOutput, kZero);
      //shooterMotor.set(ControlMode.Velocity, kZero);
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
        shooterMotor.set(ControlMode.Velocity, kShooterSpinup);
        
        if (controller.getXButton()) { //if the 'X' button is pushed, fire the balls
          conveyorState = "FireFireFire";
        }
      break;

      case "FireFireFire":
        shooterMotor.set(ControlMode.Velocity, kShooterVelocity);
        
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
        shooterMotor.set(ControlMode.Velocity, kZero);

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
          conveyorState = "move2Mid";
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
          conveyorState = "move2shooter";
        }

        if(controller.getXButton()) {
          conveyorState = "move2shooter";
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
        shooterMotor.set(ControlMode.PercentOutput, kShooterPercentSpin);
        
        if (controller.getXButton()) { //if the 'X' button is pushed, fire the balls
          conveyorState = "FireFireFire";
        }
      break;

      case "FireFireFire":
        shooterMotor.set(ControlMode.PercentOutput, kShooterPercentShoot);
        
        if (shooterMotor.getSelectedSensorVelocity() >= kShooterTarget) {
          conveyorMotor.set(-kConveyorSpeed);
          
          if (controller.getXButton() && !inGate_BB.get() && !midGate_BB.get() && !shooter_BB.get()) { //shooter runs when x is pushed and continues until x is pushed again. Do not hit x again until conveyor is empty
            conveyorState = "intake1";
          }
        }
      break;

      case "emergencyClear":
        conveyorMotor.set(kConveyorSpeed);
        centerMotor.set(kIntakeSpeed);
        rollerMotor.set(kIntakeSpeed);
        shooterMotor.set(ControlMode.Velocity, kZero);

        if (!inGate_BB.get() && !midGate_BB.get() && !shooter_BB.get()) { //shooter runs when x is pushed and continues until x is pushed again. Do not hit x again until conveyor is empty
          conveyorState = "intake1";
        }
      break;
    }
      
    //barf button
    if (controller.getLeftBumper() && controller.getRightBumper()) {
      conveyorState = "emergencyClear";
    }
  }

  double Deadband(double value) {
    if (value >= +kDeadband) { //Upper deadband
      return value;
    }
    else if (value <= -kDeadband) { //Lower deadband
      return value;
    }
    else {  //Outside deadband 
      return 0.0;
    }
  }
}