package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.Controller;
//import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableRegistry;

/**
 * This program is configured to control the 2022 FRC Robot.
 * WPILib is setup to automatically call these methods when the specific 
 * mode is enabled, don't change their names.
 */
public class Robot extends TimedRobot {
  //Controller
  XboxController controller = new XboxController(0);

  //Drive Train
    //Drive: 4 Falcon motors with integrated motor controllers

        WPI_TalonFX LeftFront = new WPI_TalonFX(1, "rio");
        WPI_TalonFX LeftRear = new WPI_TalonFX(2, "rio");
        WPI_TalonFX RightFront = new WPI_TalonFX(3, "rio");
        WPI_TalonFX RightRear = new WPI_TalonFX(4, "rio");

      //Group the left together
        //MotorControllerGroup driveleft = new MotorControllerGroup(LeftFront, LeftRear);
      //Group the Right together
         //MotorControllerGroup driveright = new MotorControllerGroup(RightFront, RightRear);
      //Drive
        //DifferentialDrive drive = new DifferentialDrive(driveleft, driveright);
  
  
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
                //Change the I2C port below to math the connection of your color sensor
                private final I2C.Port i2cPort = I2C.Port.kOnboard;
                private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

      //Shooter: Falcon

    /**
   * This method is run when the robot is first started up and should be
   * used for any initialization code.
   */
  
   @Override
    public void robotInit() {
      //These methods give names to our shuffleboard livewindow outputs
      //SendableRegistry.add(drive, "drive");
      //SendableRegistry.add(intakeMotor, "intakeMotor");
      //SendableRegistry.add(intakePneumatic, "intakePneumatic");
  }

    @Override
    public void teleopInit(){
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

     @Override
  public void robotPeriodic() {
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
     * The sensor returns a raw IR value of the infrared light detected.
     */
    double IR = m_colorSensor.getIR();

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

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
    int proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);

  }
    
     @Override
     public void teleopPeriodic() {
          //arcade drive has two parmeters, one controller axis for forward and reverse
          //and one for turning the robot left and right
              //* -1 because the inputs on xbox controllers are flipped
          //drive.arcadeDrive(-1 * controller.getLeftY(), controller.getRightX());

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
      
          

    //Drive
      // Controls:
      // Left Joystick Y-Axis: Drive robot forward and reverse
      // Right Joystick X-Axis: Turn robot right and left
    
    //Intake control
      //If the left bumper button on the controller is pressed the intake is down(forward), 
       //else it isn't pressed the intake is up(reverse)
        if (controller.getLeftBumper()){
          intakePneumatic.set(Value.kForward);
        } else {
         intakePneumatic.set(Value.kReverse);
        }

    //If the right bumper button on the controller is pressed the intake motor is spinning forward (in)
    //If the right trigger axis is pressed past 0.5 the intake motor is set to reverse (out)
    //If niether is pressed the motor stops
    //if(controller.getRightBumper()){
      //intakeMotor.set(1.0);
    //} else if (controller.getRightTriggerAxis() > 0.5){
      //intakeMotor.set(-1.0);
    //} else{
      //intakeMotor.stopMotor();
    //}
}




