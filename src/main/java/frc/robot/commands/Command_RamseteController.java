/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Subsystem_Drive;

public class Command_RamseteController extends CommandBase {
  /**
   * Creates a new RamseteCommand.
   */

  private Trajectory traj;
  private Subsystem_Drive driveSub;

  
  public Command_RamseteController(Trajectory traj, Subsystem_Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.traj = traj;
    driveSub = drive;
  }
  /*
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final RamseteCommand follow = new RamseteCommand( 
      traj, 
      driveSub::getPose, 
      new RamseteController(), 
      new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), 
      Constants.kDriveKinematics, 
      driveSub::getWheelSpeeds, 
      new PIDController(Constants.kPDriveVel, 0.0, 0.0), 
      new PIDController(Constants.kPDriveVel, 0.0, 0.0), 
      driveSub::voltageDrive, 
      driveSub);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override 
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    driveSub.DriveStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
  */
}
