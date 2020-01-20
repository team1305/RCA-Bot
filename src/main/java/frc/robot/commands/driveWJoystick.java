/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class driveWJoystick extends CommandBase {
  /**
   * Creates a new driveWJoystick.
   */

   DriveSubsystem drive;
  public driveWJoystick(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //deadbanding
    double leftStickY = RobotContainer.driver.getRawAxis(1);
    double rightStickX = RobotContainer.driver.getRawAxis(4);

    if(Math.abs(leftStickY) < Constants.leftStickDeadband) leftStickY = 0;
    if(Math.abs(rightStickX) < Constants.rightStickDeadband) rightStickX = 0;

    drive.driveWJoystick(leftStickY, rightStickX);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.halt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
