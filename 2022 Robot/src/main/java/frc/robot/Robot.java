// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Controller
  XboxController controller = new XboxController(0);

  //Drive Train
    //Drive: 4 Falcon motors with integrated motor controllers

    WPI_TalonFX LeftFront = new WPI_TalonFX(1, "rio");
    WPI_TalonFX LeftRear = new WPI_TalonFX(2, "rio");
    WPI_TalonFX RightFront = new WPI_TalonFX(3, "rio");
    WPI_TalonFX RightRear = new WPI_TalonFX(4, "rio");

//Cargo Handling

  //Intake

      //Pivot: Up/Down - Air: Double Acting Solenoid and Piston
        DoubleSolenoid intakePneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

      //Roller Drive: In/Out - Spark Max and Neo
        CANSparkMax rollerMotor = new CANSparkMax(1, MotorType.kBrushless);

      //Sensors: Intgrated hall effect for motor speed

  //Centering

      //Left and Right: Center/Eject - Spark Max and Neo
        CANSparkMax centerMotor = new CANSparkMax(2, MotorType.kBrushless);

      //Sensors: Integrated hall effect for motor speed

  // Conveyor

      //Inside robot: Up/down - Spark Max and Neo
        CANSparkMax conveyorMotor = new CANSparkMax(3, MotorType.kBrushless);

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

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
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
                String colorString;
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

    /* Ensure motor output is neutral during init */
    LeftFront.set(ControlMode.PercentOutput, 0);
    LeftRear.set(ControlMode.PercentOutput, 0);
    RightFront.set(ControlMode.PercentOutput, 0);
    RightRear.set(ControlMode.PercentOutput, 0);

/* Factory Default all hardware to prevent unexpected behaviour */
    LeftFront.configFactoryDefault();
    LeftRear.configFactoryDefault();
    RightFront.configFactoryDefault();
    RightRear.configFactoryDefault();

/* Set Neutral mode */
    LeftFront.setNeutralMode(NeutralMode.Brake);
    LeftRear.setNeutralMode(NeutralMode.Brake);
    RightFront.setNeutralMode(NeutralMode.Brake);
    RightRear.setNeutralMode(NeutralMode.Brake);

/* Configure output direction */
    LeftFront.setInverted(TalonFXInvertType.CounterClockwise);
    LeftRear.setInverted(TalonFXInvertType.CounterClockwise);
    RightFront.setInverted(TalonFXInvertType.Clockwise);
    RightRear.setInverted(TalonFXInvertType.Clockwise);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Intake control
      //If the left bumper button on the controller is pressed the intake is down(forward), 
       //else it isn't pressed the intake is up(reverse)
       if (controller.getLeftBumper()){
        intakePneumatic.set(Value.kForward);
      } else {
       intakePneumatic.set(Value.kReverse);
      }
    /* Gamepad processing */
		double forward = -1 * controller.getLeftY();
		double turn = controller.getRightX();		
		forward = Deadband(forward);
		turn = Deadband(turn);

		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
		LeftFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
		LeftRear.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
    RightFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
		RightRear.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
    }

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
