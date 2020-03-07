package frc.robot.auto_commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 *
 */
public class Auto_Turn_To_Target_Master extends Command {

	double x;
	int isuccess;

	public Auto_Turn_To_Target_Master() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);

		// requires(Robot.drive);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.drive.LowGear();
		Robot.limelight.limelightOn();
		isuccess = 0;

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		x = Robot.limelight.get_Tx();
		// x = x + Robot.limelight.getOffsetRatio();
		// SmartDashboard.putNumber("x ai loop", x);

		Robot.drive.turnRobotToAngleAuto(x);

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (Math.abs(x) <= 1) {
			// Calc Distance away so we know zone 1 or zone 2

			isuccess = isuccess + 1;


			if (isuccess >= 4) {//5
				return true;

			} else {
				return false;
			}
		}

		else{
			return false;
		}

	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.drive.DriveStop();
		Robot.drive.HighGear();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
