package frc.robot.auto_commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 *
 */
public class Auto_Turn_To_Target extends Command {
    double x;

  
	
    public Auto_Turn_To_Target() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
         requires(Robot.drive);
         requires(Robot.limelight);

    // Called just before this Command runs the first time
       }

    protected void initialize() {
		
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        x = Robot.limelight.get_Tx();
        x = x + Robot.limelight.getOffsetRatio();
        Robot.drive.turnRobotToAngle(x);


    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        boolean result;
        x = Robot.limelight.get_Tx();
        //x = x + Robot.limelight.getOffsetRatio();
        if (x > 1)
        {
                result = false;
        }
        else if (x < -1)
        {
            result = false;
        }

        else{
            result = true;
        }

    	return result;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.drive.driveStop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
