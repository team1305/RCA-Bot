package frc.robot.auto_commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 *
 */
public class Auto_Intake_On extends Command {

	
	
    public Auto_Intake_On() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
         //requires(Robot.intake);
         //requires(Robot.hopper);
             	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.intake.extendIntake();
    	//Robot.intake.enableIntake(0.5);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.intake.enableIntake(0.5);
        Robot.hopper.hopperIn(0.9);
	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	//Robot.drive.driveStop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
