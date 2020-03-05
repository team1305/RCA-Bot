/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;

//import com.revrobotics.CANEncoder;
//import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Command_Drive_With_Joystick;

/**
 * Add your docs here.
 */
public class Subsystem_Drive extends Subsystem {

  // variable for deadband
  public double dDeadband = 0.1;
  public double dSquareFactor = 1.0;
  public double dThrottleFactor = 0.5;

  // boolean for checking gear shift state
  public boolean bIsLow = false;

  // grabs drive motor information from RobotMap
  private final static WPI_TalonFX mtLeft1 = RobotMap.mtDriveLeft1;
  private final static WPI_TalonFX mtLeft2 = RobotMap.mtDriveLeft2;
  private final static WPI_TalonFX mtRight1 = RobotMap.mtDriveRight1;
  private final static WPI_TalonFX mtRight2 = RobotMap.mtDriveRight2;

  private static AHRS navx;
  private Encoder Left_wheel_encoder;
  private Encoder Right_wheel_encoder;

  double circumferenceOfWheel = 6 * Math.PI;
  double GearRatio = 0.16666; /// 11/66
  double pulsesPerRevolution = 20;
  double driveCorrectionMultiplier = 1.43; // set this equal actual distance / desired distance

  private double Kp = Constants.LIMELIGHT_KP;
  private double Ki = Constants.LIMELIGHT_KI; // 0.006
  private double Kd = 0; // 0.006
  private double Kf = Constants.LIMELIGHT_KF; // feedforward - minimum command signal

  // public PIDController limelightpid = new PIDController(Kp,Ki,Kd,Kf);

  // private AHRS navx = new AHRS(SerialPort.Port.kMXP);

  // grabs drive encoder information from RobotMap
  // private final CANEncoder enLeft1 = RobotMap.enDriveLeft1;
  // private final CANEncoder enLeft2 = RobotMap.enDriveLeft2;
  // private final CANEncoder enRight1 = RobotMap.enDriveRight1;
  // private final CANEncoder enRight2 = RobotMap.enDriveRight2;

  // grabs solenoid for gear shifting
  private final Solenoid slndShift = RobotMap.slndGearShifter;

  // private DifferentialDriveOdometry odometry;

  // creates motor controller groups for left and right motors
  public static SpeedControllerGroup scgLeft = new SpeedControllerGroup(mtLeft1, mtLeft2);
  public static SpeedControllerGroup scgRight = new SpeedControllerGroup(mtRight1, mtRight2);

  // Internal subsystem parts, declares left and right motor groups used by
  // differential drive
  DifferentialDrive drRobotDrive = new DifferentialDrive(scgLeft, scgRight);

  // sets ramprate of drive motors -- now does things!
  public Subsystem_Drive() {

    navx = new AHRS(I2C.Port.kMXP);

    // Trying to get brake mode working
    mtLeft1.setNeutralMode(NeutralMode.Brake);
    mtLeft2.setNeutralMode(NeutralMode.Brake);
    mtRight1.setNeutralMode(NeutralMode.Brake);
    mtRight2.setNeutralMode(NeutralMode.Brake);

    mtLeft1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mtRight1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    mtLeft1.configOpenloopRamp(0.2);
    mtLeft2.configOpenloopRamp(0.2);
    mtRight1.configOpenloopRamp(0.2);
    mtRight2.configOpenloopRamp(0.2);

    drRobotDrive.setDeadband(0.09); // By default, the Differential Drive class applies an input deadband of .02

    drRobotDrive.setSafetyEnabled(false);

    mtLeft1.configStatorCurrentLimit(RobotMap.currentLimitConfig, 40); // TAKE A LOOK AT CURRENT LIMITING IMPORTANT
    mtLeft2.configStatorCurrentLimit(RobotMap.currentLimitConfig, 40);
    mtRight1.configStatorCurrentLimit(RobotMap.currentLimitConfig, 40);
    mtRight2.configStatorCurrentLimit(RobotMap.currentLimitConfig, 40);
    // SmartDashboard.putNumber("dSquareFactor", dSquareFactor);
    // SmartDashboard.putNumber("dThrottleFactor", dThrottleFactor);
  }

  public double getratio_low() {
    // Convert encoder ticks to 1 inch
    double encoder_ticks = 2048;  // TODO: move these to class constents as they get reused in several locations
    double gearratio_low = 26.67;
    double gearratio_high = 11.73;
    double wheel_circumfrance = 6 * Math.PI;
    return Math.round((encoder_ticks * gearratio_low) / wheel_circumfrance);
  }

  public double getratio_high() {
    // Convert encoder ticks to 1 inch
    double encoder_ticks = 2048;
    double gearratio_low = 26.67;
    double gearratio_high = 11.73;
    double wheel_circumfrance = 6 * Math.PI;
    return Math.round((encoder_ticks * gearratio_high) / wheel_circumfrance);
  }

  public void gyroReset() {
    navx.reset();
  }

  public void driveTank(double leftValue, double rightValue) { // For AutoDriveStraight and Rotate and Curve
    // Used in Auto - AutoNewDriveStraight
    // Robot.intake.intakeOff();
    // drRobotDrive.tankDrive(leftValue, rightValue);

  }

  //
  public double gyroGetAngle() {
    // return ahrs.getAngle();
    // SmartDashboard.putNumber("Gyro Angle", navx.getAngle());
    // return navx.getAngle();
    return navx.getYaw();

  }

  public void resetEncoder() {
     //Left_wheel_encoder.reset();
     //Right_wheel_encoder.reset();

   //mtRight1.getSensorCollection().setIntegratedSensorPosition(0, 30);
    

    mtRight1.setSelectedSensorPosition(0);
    mtLeft1.setSelectedSensorPosition(0);

    // driveLeft1.getSensorCollection().setQuadraturePosition(0,10); //cimcoder
    // driveRight4.getSensorCollection().setQuadraturePosition(0,10); //cimcoder

  }

  public double getPosition() {
    double position = getEncLeftSide();
    return position;
  }

  public static double getDistance() {
    double left_wheel_rot = -1 * getEncLeftSide();
    double right_wheel_rot = getEncRightSide();
    SmartDashboard.putNumber("Left Encoder", left_wheel_rot);
    SmartDashboard.putNumber("Right Encoder", right_wheel_rot);
    double average_wheel_rot = right_wheel_rot; // (left_wheel_rot + right_wheel_rot) / 2;


    //double average_wheel_rot = mtRight1.getSensorCollection().getIntegratedSensorPosition();
  
    return average_wheel_rot;

  }

  public double getDistance_Left() {
    double left_wheel_rot = -1 * getEncLeftSide();
    double right_wheel_rot = getEncRightSide();
    SmartDashboard.putNumber("Left Encoder", left_wheel_rot);
    SmartDashboard.putNumber("Right Encoder", right_wheel_rot);
   
    double average_wheel_rot = (left_wheel_rot + right_wheel_rot) / 2;
    return left_wheel_rot;

  }

  public double getDistance_Right() {
    double left_wheel_rot = -1 * getEncLeftSide();
    double right_wheel_rot = getEncRightSide();
    SmartDashboard.putNumber("Left Encoder", left_wheel_rot);
    SmartDashboard.putNumber("Right Encoder", right_wheel_rot);
    double average_wheel_rot = (left_wheel_rot + right_wheel_rot) / 2;
    return right_wheel_rot;

  }

  public void setRightSide(double speed) {
    scgRight.set(speed);

  }

  public void setLeftSide(double speed) {
    scgLeft.set(speed);
  }

  public double getRightSide() {
    double currentSpeed = scgRight.get();
    return currentSpeed;
  }

  public static double getEncRightSide() {
    return mtRight1.getSelectedSensorPosition();
  }

  public static double getEncLeftSide() {
    return mtLeft1.getSelectedSensorPosition();
  }

  public double getLeftSide() {
    double currentSpeed = scgLeft.get();
    return currentSpeed;
  }

  @Override
  public void initDefaultCommand() {
    // unless interupted the default command will allow driver to drive with
    // joystick
    setDefaultCommand(new Command_Drive_With_Joystick());
  }

  // creates a deadband for the joystick so that the robot does not spin
  // when nobody is touching the controls
  private double JoystickDeadBand(double input) {
    if (Math.abs(input) < dDeadband)
      return 0;
    else if (input > 0)
      return Math.pow(((input - dDeadband) * (1 / (1 - dDeadband))), dSquareFactor);
    else if (input < 0)
      return ((Math.pow(((Math.abs(input) - dDeadband) * (1 / (1 - dDeadband))), dSquareFactor)) * -1);
    else
      return 0;
  }

  private double ThrottleScale(double throttle, double input) {
    return (JoystickDeadBand(input) * (1 - (throttle * dThrottleFactor)));
  }

  public void curvaturedrive(double xspeed, double zrotation) {
    drRobotDrive.curvatureDrive(xspeed, zrotation, true);
  }

  // creates a driving function using specified joystick
  public void driveWithJoystick(Joystick stick) {

    // creates variables for joystick x and y values
    double zRotation = ThrottleScale(Math.abs(stick.getY() * 1), stick.getRawAxis(4) * -1);
    double xSpeed = JoystickDeadBand(stick.getY() * 1);
    double dist = getDistance();
    double angle = gyroGetAngle();
    // uses joystick to do driving thing

    SmartDashboard.putNumber("drive stick  speed", xSpeed);
    SmartDashboard.putNumber("drive stick  rotation", zRotation);
    SmartDashboard.putNumber("RAW Gyro Angle", angle);
    SmartDashboard.putNumber("RAW Encoder Right", getDistance());
    drRobotDrive.curvatureDrive(xSpeed, zRotation, true);

  }

  // stops the drive train
  public void DriveStop() {
    setLeftSide(0);
    setRightSide(0);
    
  }

  public void DriveTank(double leftValue, double rightValue) {
    drRobotDrive.tankDrive(leftValue, rightValue);
  }

  public void arcadedrive(double speed, double rotation) {
    Robot.drive.setRightSide(speed);
    Robot.drive.setLeftSide(-speed);
  }

  // sets the speed for climbing drive thingy
  public void ClimbSpeed() {
    drRobotDrive.arcadeDrive(-0.7, 0);
  }

  // shifts drive train to low gear
  public void HighGear() {
    this.slndShift.set(false);
    bIsLow = false;
    SmartDashboard.putBoolean("Gear", bIsLow);
  }

  // shifts drive train to high gear
  public void LowGear() {
    this.slndShift.set(true);
    bIsLow = true;
    SmartDashboard.putBoolean("Gear", bIsLow);
  }

  public boolean IsLow() {
    return bIsLow;
  }

  // toggles gear state
  public void toggleGear() {
    if (bIsLow) {
      HighGear();
    } else {
      LowGear();
    }
    SmartDashboard.putBoolean("Gear", bIsLow);
  }

  public void turnRobotToAngle(double x){

    if (Robot.limelight.is_Target()) {
      double left_command;
      double right_command;

      left_command = Robot.drive.getLeftSide();
      right_command = Robot.drive.getRightSide();

      double heading_error = -x;
      double steering_adjust = 0.0f;
           if (x > 1)
           {
                   steering_adjust = Kp*heading_error + Kf;
           }
           else if (x < -1)
           {
                   steering_adjust = Kp*heading_error - Kf;
           }
           else{
             steering_adjust = 0;
           }


      left_command += steering_adjust;
      right_command -= steering_adjust;

      SmartDashboard.putNumber("Left Command", left_command);
      SmartDashboard.putNumber("Right Command", right_command);
      SmartDashboard.putNumber("Steering Adjust", steering_adjust);
      SmartDashboard.putNumber("X", x);


      if (left_command > 0.75){
        left_command = 0.75;
      }

      if (right_command > 0.75){
        right_command = 0.75;
      }

      Robot.drive.setLeftSide(-left_command);
      Robot.drive.setRightSide(right_command);
    }
  }

  public void turnRobotToAngleAuto(double x) {

    if (Robot.limelight.is_Target()) {
      double left_command;
      double right_command;

      left_command = Robot.drive.getLeftSide();
      right_command = Robot.drive.getRightSide();

      double heading_error = -x;
      double steering_adjust = 0.0f;
      if (x > 1) {
        steering_adjust = Kp * heading_error + Kf;
      } else if (x < -1) {
        steering_adjust = Kp * heading_error - Kf;
      } else {
        steering_adjust = 0;
      }

      left_command += steering_adjust;
      right_command -= steering_adjust;

      SmartDashboard.putNumber("Left Command", left_command);
      SmartDashboard.putNumber("Right Command", right_command);
      SmartDashboard.putNumber("Steering Adjust", steering_adjust);
      SmartDashboard.putNumber("X", x);

      if (left_command > 0.15) {
        left_command = 0.15;
      }

      if (right_command > 0.15) {
        right_command = 0.15;
      }

      Robot.drive.setLeftSide(-left_command);
      Robot.drive.setRightSide(right_command);
    }
  }

  public static double getRoll() {
    double angle = navx.getRoll();
    return angle;
  }

 
  

  /*
  public void turnRobotToAnglePID(double x){

    if (Robot.limelight.is_Target()) {
      double left_command;
      double right_command;

      left_command = Robot.drive.getLeftSide();
      right_command = Robot.drive.getRightSide();

      double heading_error = -x;
      double steering_adjust = 0.0f;
           if (Math.abs(x) > (x+1))
           {
                   steering_adjust = limelightpid.calculate(heading_error);
           }

           else{
             steering_adjust = 0;
           }


      left_command = steering_adjust;
      right_command = steering_adjust;

      SmartDashboard.putNumber("Left Command", left_command);
      SmartDashboard.putNumber("Right Command", right_command);
      SmartDashboard.putNumber("Steering Adjust", steering_adjust);
      SmartDashboard.putNumber("X", x);


      if (left_command > 0.75){
        left_command = 0.75;
      }

      if (right_command > 0.75){
        right_command = 0.75;
      }

      Robot.drive.setLeftSide(-left_command);
      Robot.drive.setRightSide(right_command);
    }
    
  }
  */


  

}

