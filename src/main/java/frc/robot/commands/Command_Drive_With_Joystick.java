/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
//import frc.robot.subsystems.Subsystem_Drive;

public class Command_Drive_With_Joystick extends Command {
  public Command_Drive_With_Joystick() {
    
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { 
    //SmartDashboard.putString("executing", "Executing");
    Robot.drive.driveWithJoystick(Robot.oi.getJoystickDriver());
    SmartDashboard.putNumber("Gyro Angle", Robot.drive.gyroGetAngle());
    SmartDashboard.putNumber("RIGHT SIDE RPM 1 TESTING", Robot.drive.getRightSide1RPM());
    SmartDashboard.putNumber("LEFT SIDE RPM 1 TESTING", Robot.drive.getLeftSide1RPM());
    SmartDashboard.putNumber("RIGHT SIDE RPM 2 TESTING", Robot.drive.getRightSide2RPM());
    SmartDashboard.putNumber("LEFT SIDE RPM 2 TESTING", Robot.drive.getLeftSide2RPM());
    //SmartDashboard.putNumber("TESTING", Robot.drive.gyroGetAngle());


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
    end();
  }
}