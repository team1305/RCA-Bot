/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Command_Limelight_Initiation_Line extends Command {
  public Command_Limelight_Initiation_Line() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drive);
    requires(Robot.limelight);

    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //TURN ON LIMELIGHT AND SELECT THE CORRECT PIPELINE

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    NetworkTable table = Robot.limelight.getLimelightValues();
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    double x = tx.getDouble(0.0);
    //double y = ty.getDouble(0.0);


    double Kp = -0.1f;
    double min_command = 0.05f;

    //float tx = table->GetNumber("tx");
    double left_command;
    double right_command;

    left_command = Robot.drive.getLeftSide();
    right_command = Robot.drive.getRightSide();



    double heading_error = -x;
    double steering_adjust = 0.0f;
            if (x > 1.0)
            {
                    steering_adjust = Kp*heading_error - min_command;
            }
            else if (x < 0.0)
            {
                    steering_adjust = Kp*heading_error + min_command;
            }


    left_command += steering_adjust;
    right_command -= steering_adjust;


    Robot.drive.setLeftSide(left_command);
    Robot.drive.setRightSide(right_command);




  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
