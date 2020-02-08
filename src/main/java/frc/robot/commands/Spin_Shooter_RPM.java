/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
//import frc.robot.subsystems.ShooterRPM;
import frc.robot.subsystems.ShooterRPM;

public class Spin_Shooter_RPM extends Command {

  double RPM;

  public Spin_Shooter_RPM(double RPM) {

    //requires(Robot.shooterrpm);
    addRequirements(Robot.shooterrpm);
    this.RPM = RPM;
  }

  private void addRequirements(ShooterRPM shooterrpm) {
  }
 
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    Robot.shooterrpm.setShooterRPM(RPM);

  }



  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.shooterrpm.setShooterRPM(0);


  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
