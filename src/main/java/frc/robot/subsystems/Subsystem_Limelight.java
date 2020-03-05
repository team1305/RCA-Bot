/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;


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
  private double Kd = 0; // 0.006
  private double Kf = Constants.LIMELIGHT_KF;  //feedforward - minimum command signal

  //public PIDController limelightpid = new PIDController(Kp,Ki,Kd,Kf);



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

  public double getOffsetRatio(){
    double tshort = table.getEntry("tshort").getDouble(0.0);
    double tlong = table.getEntry("tlong").getDouble(0.0);
    double tresult = tlong-tshort;
    double tdistance = Robot.limelight.getDistance();
    double result = 0;
    if ((tdistance >= 192) && (tdistance <= 242)){
      if(Math.abs(tresult) >= 2){
        if (Robot.drive.gyroGetAngle() >= 2){
          result = 0 - tresult;
        }

        else if (Robot.drive.gyroGetAngle() <= -2){
          result = tresult;
        }

        else{
          result = tresult;
        }
      }
    }
    return result;   
  }




  public double getDistance(){
    double distance = 0;
    double height_of_limelight = 26.5; //Inches
    double height_of_target = 98; //Inches
    double angle_of_limelight = 25; // Degrees

    if (is_Target()){
      double tanValue = Math.tan((angle_of_limelight + get_Ty())*(Math.PI/180));
      SmartDashboard.putNumber("Tan Value", tanValue);
      SmartDashboard.putNumber("The angle", angle_of_limelight + get_Ty());
      distance = (height_of_target - height_of_limelight) / Math.tan((angle_of_limelight + get_Ty())*(Math.PI/180));
      distance = Math.abs(distance);
    }
    return distance;
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

  public void limelightOff(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  public void limelightOn(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  public boolean isLimelightOn(){
    int isOn = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getNumber(1).intValue();
    if (isOn == 3){
      return true;
    } 
    else{
      return false;
    }
  }

  /*
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
  */



  

}
