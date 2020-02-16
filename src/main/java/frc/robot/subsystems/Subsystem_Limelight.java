/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * Add your docs here.
 */
public class Subsystem_Limelight extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public NetworkTable table;
  public NetworkTableEntry tx;
  private double left_command;
  private double right_command;
  private double x, thor; //Limelight Values
  private double Kp = Constants.LIMELIGHT_KP;
  private double Ki = Constants.LIMELIGHT_KI; // 0.006
  private double Kf = Constants.LIMELIGHT_KF;  //feedforward - minimum command signal



  public Subsystem_Limelight() {
    table = getLimelightValues();
  }

  public double get_Tx(){
    return table.getEntry("tx").getDouble(0.0);
  }


  public boolean is_Target(){
    if (table.getEntry("tv").getDouble(0) == 1) {
      return true;
   } else {
      return false;
   }
  }

  public double get_Thor(){
    return table.getEntry("thor").getDouble(0.0);
  }

  public double get_Ty(){
    return table.getEntry("ty").getDouble(0.0);
  }


  public double getDistance(){
    double distance = 0;
    double height_of_limelight = 26.5; //Inches
    double height_of_target = 98; //Inches
    double angle_of_limelight = 25; // Degrees

    if (is_Target()){
      double tanValue = Math.tan(angle_of_limelight + get_Ty());
      SmartDashboard.putNumber("Tan Value", tanValue);
      distance = (height_of_target - height_of_limelight) / Math.tan(angle_of_limelight + get_Ty());
      distance = Math.abs(distance);
    }
    return distance;
  }

  public void turnRobotToAnlge(){

      left_command = Robot.drive.getLeftSide();
      right_command = Robot.drive.getRightSide();

      double heading_error = -x;
      double steering_adjust = 0.0f;
           if (x > 1.5)
           {
                   steering_adjust = Kp*heading_error + Kf;
           }
           else if (x < -1.5)
           {
                   steering_adjust = Kp*heading_error - Kf;
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

    public void trackToDistance(){
        double KpAim = -0.1f;
        double KpDistance = -0.1f;
        double min_aim_command = 0.05f;

        double heading_error = -get_Tx();
        double distance_error = -get_Ty(); //Plus the inverse of expected value for distance to position
        double tx = get_Tx();
        double steering_adjust = 0.0f;

        if (tx > 1)
        {
                steering_adjust = KpAim*heading_error - min_aim_command;
        }
        else if (tx < -1)
        {
                steering_adjust = KpAim*heading_error + min_aim_command;
        }

        double distance_adjust = KpDistance * distance_error;

        left_command += steering_adjust + distance_adjust;
        right_command -= steering_adjust + distance_adjust;

        if (left_command > 0.75){
          left_command = 0.75;
        }
  
        if (right_command > 0.75){
          right_command = 0.75;
        }
  
        Robot.drive.setLeftSide(-left_command);
        Robot.drive.setRightSide(right_command);
    }

  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

  }


  public NetworkTable getLimelightValues() {
 
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    //double x = tx.getDouble();


    //read values periodically

    //post to smart dashboard periodically
    //SmartDashboard.putNumber("LimelightX", tx);
    //SmartDashboard.putNumber("LimelightY", ty);

    return table;

  }


  

}
