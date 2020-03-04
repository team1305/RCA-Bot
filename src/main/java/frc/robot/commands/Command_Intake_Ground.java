/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Command_Intake_Ground extends Command {
  public Command_Intake_Ground() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.intake);
    requires(Robot.hopper);
    requires(Robot.elevator);
    requires(Robot.shooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.intake.extendIntake();

    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.intake.enableIntake(0.9);
    Robot.hopper.hopperIn(0.9);
    //Robot.elevator.elevatorUp(0.1);
    Robot.shooter.setShooterSpeed(-0.05);
  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    
    Robot.intake.retractIntake();
    Robot.intake.stopIntake();
    Robot.hopper.hopperStop();
    Robot.elevator.elevatorStop();
    Robot.shooter.setShooterSpeed(0);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
