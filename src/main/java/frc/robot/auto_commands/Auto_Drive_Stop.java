package frc.robot.auto_commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 *
 */
public class Auto_Drive_Stop extends Command {

  
	
    public Auto_Drive_Stop() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	 requires(Robot.drive);

    // Called just before this Command runs the first time
       }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drive.driveStop();

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
