/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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



  public Subsystem_Limelight() {
    table = Robot.limelight.getLimelightValues();
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
    double height_of_limelight = 26; //Inches
    double height_of_target = 185; //Inches
    double angle_of_limelight = 45; // Degrees

    if (is_Target()){
      distance = (height_of_target - height_of_limelight) / Math.tan(angle_of_limelight + get_Ty());

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


  

}
