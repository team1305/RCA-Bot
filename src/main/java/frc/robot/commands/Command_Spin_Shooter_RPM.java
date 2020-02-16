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

public class Command_Spin_Shooter_RPM extends Command {
  double shooterRPMValue = 5000;
  boolean BuseSmartDashboard;

  double RPM;

  public Command_Spin_Shooter_RPM(double RPM, boolean useSmartDashboard) {

    BuseSmartDashboard = useSmartDashboard;
        //requires(Robot.shooterrpm);
    requires(Robot.shooter);
    this.RPM = RPM;
  }

 
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.shooter.getPidFromDashboard();//This gets value
    //Robot.shooter.compareUpdatePid();//This compares and updates PID values
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    /*
    if (BuseSmartDashboard){
      double shooterRPMValue = SmartDashboard.getNumber("Shooter RPM", 5000);
      Robot.shooter.setShooterRPM(shooterRPMValue);
    }
    else{
      Robot.shooter.setShooterRPM(RPM);
    }
    */
    Robot.shooter.setShooterRPM(RPM);

           
    SmartDashboard.putNumber("shooterRPM", Robot.shooter.getShooterRPM() );
           if (Robot.shooter.getShooterRPM() >= RPM) {
              // We are at speed, Turn on feeders 
              Robot.elevator.elevatorUp(0.3);
              Robot.hopper.hopperOut(0.2);
              Robot.intake.enableIntake(0.2);
           }

  }



  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.shooter.setShooterSpeed(0);


  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
