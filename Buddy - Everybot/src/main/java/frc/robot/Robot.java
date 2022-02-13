package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.util.sendable.SendableRegistry;

/**
 * This program is configured to control "Buddy" robot.
 * Buddy is a 4-Cim drive train with a seat attached.
 */
public class Robot extends TimedRobot {
  //Controller
  XboxController controller = new XboxController(0);

  /*  Drive Train - Create 4 motor controllers, two motor controller groups, one for each side
  *   and a DifferentialDrive which lets us easily control the drive train
  */
      VictorSP leftmotorwhite = new VictorSP(0);
      VictorSP leftmotoryellow = new VictorSP(3);
      VictorSP rightmotorgreen = new VictorSP(4);
      VictorSP rightmotorblue = new VictorSP(2);

      // Groups all of the left-side motors together
      MotorControllerGroup driveleft = new MotorControllerGroup(leftmotorwhite, leftmotoryellow);
      //Groups all of the right-side motors together
      MotorControllerGroup driveright = new MotorControllerGroup(rightmotorblue, rightmotorgreen);
      //Drive
      DifferentialDrive drive = new DifferentialDrive(driveleft, driveright);


    /**
   * This method is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //These methods give names to our shuffleboard livewindow outputs
    SendableRegistry.add(drive, "drive");
  }

  /**
   * This method is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //arcade drive has two parmeters, one controller axis for forward and reverse
    //and one for turning the robot left and right
    drive.arcadeDrive(-1 * controller.getLeftY(), controller.getRightX());
  }
}