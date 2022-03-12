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
//import edu.wpi.first.wpilibj.Controller;
//import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.fasterxml.jackson.databind.introspect.TypeResolutionContext.Empty;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;

//import edu.wpi.first.util.sendable.SendableRegistry;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String k_taxiAuto = "Taxi Auto";
  private static final String k_ballShootingAuto = "Ball Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Controller
  XboxController controller = new XboxController(0);

  //Tells the robot what alliance it
  String allianceColor = DriverStation.getAlliance().toString();

  //Color String
  String colorString;

  //Timer
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

      //Pivot: Up/Down - Air: Double Acting Solenoid and Piston
        DoubleSolenoid intakePneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

      //Roller Drive: In/Out - Spark Max and Neo
        CANSparkMax rollerMotor = new CANSparkMax(5, MotorType.kBrushless);

      //Sensors: Intgrated hall effect for motor speed

  //Centering

      //Left and Right: Center/Eject - Spark Max and Neo
        CANSparkMax centerMotor = new CANSparkMax(6, MotorType.kBrushless);

      //Sensors: Integrated hall effect for motor speed

  // Conveyor

      //Inside robot: Up/down - Spark Max and Neo
        CANSparkMax conveyorMotor = new CANSparkMax(7, MotorType.kBrushless);

      //Sensors: Integrated hall effect for motor speed +

          //In Gate: Beam Break - DIO ports
          DigitalInput inGate_BB = new DigitalInput(1);
          //Mid Gate: Beam Break
          DigitalInput midGate_BB = new DigitalInput(2);
          //Shooter Gate: Beam Break
          DigitalInput shooter_BB = new DigitalInput(3);
          //Intake: Color Sensor
            private final I2C.Port i2cPort = I2C.Port.kOnboard;
            private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
            private final ColorMatch m_colorMatcher = new ColorMatch();

            //Adjust RBG valies based on scanning ball with shuffleboard
            Color kRedTarget = new Color(0.561, 0.232, 0.114);
            Color kBlueTarget = new Color(0.143, 0.427, 0.429);
            Color kGreenTarget = new Color(0.197, 0.561, 0.240);
            Color kYellowTarget = new Color(0.361, 0.524, 0.113);

        
  //Shooter: Falcon
    WPI_TalonFX shooterMotor = new WPI_TalonFX(8, "rio");

    //convayer states
    //String conveyorState = "intake1"; 

  //Climber

    //Clamps
     DoubleSolenoid clampPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    //Hooks - Reach
      DoubleSolenoid pneumaticreach = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    //Hooks - Tilt
      DoubleSolenoid pneumatictilt = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Taxi Auto", k_taxiAuto);
    m_chooser.addOption("Ball Auto", k_ballShootingAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);   
   }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
            
          //Color Sensor
              /**
              * The method GetColor() returns a normalized color value from the sensor and can be
              * useful if outputting the color to an RGB LED or similar. To
              * read the raw color, use GetRawColor().
              * 
              * The color sensor works best when within a few inches from an object in
              * well lit conditions (the built in LED is a big help here!). The farther
              * an object is the more light from the surroundings will bleed into the 
              * measurements and make it difficult to accurately determine its color.
              */
              Color detectedColor = m_colorSensor.getColor();

               /**
                * Run the color match algorithm on our detected color
                */
                
                ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

                if (match.color == kBlueTarget) {
                  colorString = "Blue";
                } else if (match.color == kRedTarget) {
                  colorString = "Red";
                } else if (match.color == kGreenTarget) {
                  colorString = "Green";
                } else if (match.color == kYellowTarget) {
                  colorString = "Yellow";
                } else {
                  colorString = "Unknown";
                }
              
              /**
              * The sensor returns a raw IR value of the infrared light detected.
              */
              //double IR = m_colorSensor.getIR();

              /**
              * Open Smart Dashboard or Shuffleboard to see the color detected by the 
              * sensor.
              */
              SmartDashboard.putNumber("Red", detectedColor.red);
              SmartDashboard.putNumber("Green", detectedColor.green);
              SmartDashboard.putNumber("Blue", detectedColor.blue);
              //SmartDashboard.putNumber("IR", IR);
              SmartDashboard.putNumber("Confidence", match.confidence);
              SmartDashboard.putString("Detected Color", colorString);

              /**
              * In addition to RGB IR values, the color sensor can also return an 
              * infrared proximity value. The chip contains an IR led which will emit
              * IR pulses and measure the intensity of the return. When an object is 
              * close the value of the proximity will be large (max 2047 with default
              * settings) and will approach zero when the object is far away.
              * 
              * Proximity can be used to roughly approximate the distance of an object
              * or provide a threshold for when an object is close enough to provide
              * accurate color values.
              */
              //int proximity = m_colorSensor.getProximity();

              //SmartDashboard.putNumber("Proximity", proximity);
        }
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
    ShooterTimer.reset();
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    //tells the robot what alliance color we are
    allianceColor = DriverStation.getAlliance().toString();
  }

  /** This function is called periodically during autonomous. */
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
        LeftFront.set(0.0);
        LeftRear.set(0.0);
        RightFront.set(0.0);
        RightRear.set(0.0);
        shooterMotor.set(ControlMode.Velocity, 0.0);
        conveyorMotor.set(0.0);
        break;

      case k_ballShootingAuto:
         //drive to the goal
         LeftFront.set(1.0); //depending on what direction the robot is facing negative or positive
         LeftRear.set(1.0);
         RightFront.set(1.0);
         RightRear.set(1.0);
         shooterMotor.set(ControlMode.Velocity, 1.0); //change 1.0 to actual shooter velocity 100%
         if (LeftFront.getSelectedSensorPosition() >= 1.0) { //change 1.0 to proper value
          m_autoSelected = "FireFireFire";
         }
         //move through states to shoot
         //back out of taxi area
      break;

      case "FireFireFire":
      ShooterTimer.start();
      //if falcon is up to speed
      shooterMotor.set(ControlMode.Velocity, 1.0); //Change 1.0 to actual velocity when known
      shooterMotor.getSelectedSensorVelocity(); 
      if (shooterMotor.getSelectedSensorVelocity() >= 1.0) {// Change 1.0 to acutal velocity when known
        conveyorMotor.set(1.0);
        if (ShooterTimer.get() > 3.0) { //shooter runs until empty. Change 3 to proper length
          m_autoSelected = "reverseTaxi2";
        }
      }
      break;

      case "reverseTaxi2":
      LeftFront.set(1.0); //depending on what direction the robot is facing negative or positive
      LeftRear.set(1.0);
      RightFront.set(1.0);
      RightRear.set(1.0);
      shooterMotor.set(ControlMode.Velocity, 0.0);
      conveyorMotor.set(0.0);
      if (LeftFront.getSelectedSensorPosition() >= 1.0) { //change 1.0 to proper value
       m_autoSelected = "autoEnd";
      }
      break;

    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    allianceColor = DriverStation.getAlliance().toString();

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
    LeftRear.setNeutralMode(NeutralMode.Coast);
    RightFront.setNeutralMode(NeutralMode.Coast);
    RightRear.setNeutralMode(NeutralMode.Coast);
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

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Drivetrain Control
    /* Gamepad processing */                 
		//double forward = -1.0 * Math.pow(controller.getLeftY(), 2.0);
		//double turn = Math.pow(controller.getRightX(), 2.0);
    double forward = -1.0 * controller.getLeftY();
		double turn = controller.getRightX();		
		forward = Deadband(forward);
		turn = Deadband(turn);

		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
		LeftFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
		LeftRear.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
    RightFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		RightRear.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);

//Climber Control
    if (controller.getPOV() == 180) {
      pneumaticreach.set(Value.kForward); //up on dpad extends hooks
    }
    if (controller.getPOV() == 0) {
      pneumaticreach.set(Value.kReverse); //down on dpad retracts hooks
    }
    if (controller.getPOV() == 90) {
      pneumatictilt.set(Value.kForward); // right leans hooks back
    }                              
    if (controller.getPOV() == 270) {
      pneumatictilt.set(Value.kReverse); //left brings hooks back upright
    }
    if (controller.getRawButtonPressed(7) && controller.getRawButtonPressed(7)) {
      clampPneumatic.set(Value.kReverse);  //back and start open clamps
    }
    if (controller.getYButton()) {
      clampPneumatic.set(Value.kForward); //Y closes clamps remove
      }
    if (controller.getAButtonPressed()) {//a button centermotor
      centerMotor.set(-1.0);
    }
    if (controller.getAButtonReleased()) {
      centerMotor.set(0.0);
    }
    if (controller.getRawButtonPressed(2)) { //B poke
      intakePneumatic.set(Value.kForward);
    }
    if (controller.getRawButtonReleased(2)) { // let go of b unpoke
      intakePneumatic.set(Value.kReverse);
    }
    if (controller.getRawButtonPressed(3)) { // X button shoot
      shooterMotor.set(ControlMode.PercentOutput, 0.84);
    }
    if (controller.getRawButtonReleased(3)) { // let go of x stop shooter
      shooterMotor.set(ControlMode.PercentOutput, 0.0);
    }
    if (controller.getLeftBumperPressed()) { //left bumper intake
      rollerMotor.set(-1.0);
    }
    if (controller.getLeftBumperReleased()) { //let go of left bumper stop roller motor
      rollerMotor.set(0.0);
    }
    if (controller.getLeftBumperPressed() && controller.getBButtonPressed()) { //left bumper + back barf
      rollerMotor.set(1.0);
    }
    if (controller.getLeftBumperReleased() && controller.getBButtonReleased()) { //let go of left bumper + back barf stop
      rollerMotor.set(0.0);
    }
   // if (controller.getLeftTriggerAxis(0.5)) {//left trigger runs intake in reverse
    //  rollerMotor.set(1.0);
    //}
    //if (controller.getLeftTriggerAxis(0) {//let go of trigger to stop
   //   rollerMotor.set(0.0);
   // }
    if (controller.getRightBumperPressed()) {  // right bumper conveyor on
      conveyorMotor.set(-0.3);
    }
    if (controller.getRightBumperReleased()) { //let go of right bumper conveyor off
      conveyorMotor.set(0.0);
    }
    if (controller.getRightBumperPressed() && controller.getBButtonPressed()) {  // right bumper + back conveyor out
      conveyorMotor.set(-0.3);
    }
    if (controller.getRightBumperReleased() && controller.getBButtonReleased()) { //let go of right bumper + back conveyor out off
      conveyorMotor.set(0.0);
    }
    //if (controller.getRightTriggerAxis()) {
     // conveyorMotor.set(1.0);
    //}
    //if (controller.getRightTriggerAxis()) {
     // conveyorMotor.set(0.0)
    //}
  }

  //Conveyor Control
  /** 
      switch (conveyorState) {
            case "intake1": //what will execute while conveyor is empty
                //Intake control
                 //If the left bumper button on the controller is pressed the intake is down(forward), 
                  //else it isn't pressed the intake is up(reverse)
                  if (controller.getLeftBumper()){
                    intakePneumatic.set(Value.kForward);
                  if (controller.getAButton()){
                    rollerMotor.set(-1.0); //if they hit 'A' while the left bumper is pressed, it will eject, otherwise it will intake
                    centerMotor.set(-1.0);
                  } else {
                    rollerMotor.set(1.0);
                    centerMotor.set(1.0);
                  }
                  } else {
                    intakePneumatic.set(Value.kReverse);
                    rollerMotor.set(0.0);
                    centerMotor.set(0.0);
                  }

                  //if the beam break is broken, then the state moves to Stage2Color to run conveyor
                  if (inGate_BB.get()) {
                    conveyorState = "move2color1";
                  }
            break;
            
            case "move2color1": 
              //lift intake and stop roller motor
              intakePneumatic.set(Value.kReverse);
              rollerMotor.set(0.0);

              //run conveyor
              conveyorMotor.set(1.0);

              //if not in gate
              if (!inGate_BB.get()) {
                conveyorState = "Judgecolor1";
              }
            break; 
            
            case "Judgecolor1": 
            //stop conveyor
            conveyorMotor.set(0.0);

            //check the color sensor - must match the alliance color
            if (allianceColor == colorString) {
              conveyorState = "move2Mid";
            } else {
              conveyorState = "Ejectball1";
            }
            break;
            
            case "move2Mid": 
            conveyorMotor.set(1.0);
            if (midGate_BB.get()) {
              conveyorState = "Intake2";
            } 
            break;
            
            case "Intake2": 
            conveyorMotor.set(0.0);
                  //Intake control
                  //If the left bumper button on the controller is pressed the intake is down(forward), 
                  //else it isn't pressed the intake is up(reverse)
                  if (controller.getLeftBumper()){ //take pneumatics completely out if actuation is removed to avoid if statment issues
                    intakePneumatic.set(Value.kForward);
                  if (controller.getAButton()){
                    rollerMotor.set(-1.0); //if they hit 'A' while the left bumper is pressed, it will eject, otherwise it will intake
                    centerMotor.set(-1.0);
                  } else {
                    rollerMotor.set(1.0);
                    centerMotor.set(1.0);
                  }
                  } else {
                    intakePneumatic.set(Value.kReverse);
                    rollerMotor.set(0.0);
                    centerMotor.set(0.0);
                  }

                  if (inGate_BB.get()) {
                    conveyorState = "move2Color2";
                  }

                  if(controller.getXButton()) {
                    conveyorState = "move2shooter";
                  }
            break;
            
            case "move2Color2": 
              //lift intake and stop roller motor
              intakePneumatic.set(Value.kReverse);
              rollerMotor.set(0.0);

              //run conveyor
              conveyorMotor.set(1.0);

              //if not in gate
              if (!inGate_BB.get()) {
                conveyorState = "Judge2color2";
              }
            break;
            
            case "Judge2color2": 
            //stop conveyor
            conveyorMotor.set(0.0);

            //check the color sensor - must match the alliance color
            if (allianceColor == colorString) {
              conveyorState = "move2shooter";
            } else {
              conveyorState = "Ejectball2";
            }            
            break;
            
            case "move2shooter":
            conveyorMotor.set(1.0);
            if (shooter_BB.get()) {
              conveyorState = "spinup";
            }
            break;
            
            case "spinup": 
            conveyorMotor.set(0.0);
            shooterMotor.set(ControlMode.Velocity, 0.6); //change .6 to be 60% of full speed
            //if the 'X' button is pushed, fire the balls
            if (controller.getXButton()) {
              conveyorState = "FireFireFire";
            }
            break;

            case "FireFireFire":
                        //if falcon is up to speed
                        shooterMotor.set(ControlMode.Velocity, 1.0); //Change 1.0 to actual velocity when known
                        shooterMotor.getSelectedSensorVelocity(); 
                        if (shooterMotor.getSelectedSensorVelocity() >= 1.0) {// Change 1.0 to acutal velocity when known
                          conveyorMotor.set(1.0);
                          if (controller.getXButton() && !inGate_BB.get() && !midGate_BB.get() && !shooter_BB.get()) { //shooter runs when x is pushed and continues until x is pushed again. Do not hit x again until conveyor is empty
                            conveyorState = "intake1";
                          }
                        }
            break;

            case "Ejectball1": 
              intakePneumatic.set(Value.kForward);
              conveyorMotor.set(-1.0);
              centerMotor.set(-1.0);
              rollerMotor.set(-1.0);
              if (!inGate_BB.get() && !midGate_BB.get() && !shooter_BB.get()) { //shooter runs when x is pushed and continues until x is pushed again. Do not hit x again until conveyor is empty
                conveyorState = "intake1";
              }
            break;

            case "Ejectball2a":
              intakePneumatic.set(Value.kForward);
              conveyorMotor.set(-1.0);
              centerMotor.set(-1.0);
              rollerMotor.set(-1.0);
              if (inGate_BB.get()) {
                conveyorState = "Ejectball2b";
              }
            break;

            case "EjectBall2b":
              intakePneumatic.set(Value.kForward);
              conveyorMotor.set(-1.0);
              centerMotor.set(-1.0);
              rollerMotor.set(-1.0);
              if (!inGate_BB.get()) {
                conveyorState = "intake2";
              }

            case "emergencyClear":
              intakePneumatic.set(Value.kForward);
              conveyorMotor.set(-1.0);
              centerMotor.set(-1.0);
              rollerMotor.set(-1.0);
              shooterMotor.set(ControlMode.Velocity, 0.0);
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
*/

	/** Deadband 5 percent, used on the gamepad */
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */ 
        return 0;    
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
