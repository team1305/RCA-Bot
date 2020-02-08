/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------import frc.robot.Robot;
------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Command_Spin_Shooter extends Command {

  public Command_Spin_Shooter() {   
    requires(Robot.shooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //bIsFinished = false;
    Robot.shooter.ShooterSpeed(0.75);//THIS IS THE VALUE YOU WANT TO CHANGE FOR SHOOTER SPEED
    
    //bIsFinished = true;

    //SmartDashboard.putNumber("Shooter_Speed", Robot.shooterTwo.currentRPM());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
     return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.shooter.ShooterStop();   
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
