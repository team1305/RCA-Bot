package frc.robot.auto_commands;

import java.text.SimpleDateFormat;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 *
 */
public class Auto_Finished extends Command {

    public Auto_Finished() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drive.DriveStop();
    //	Robot.elevator.Stage1Stop();
    //	Robot.intake.intakeOff();
    //	Robot.arm.ArmUp();
    	    	
    //   	Robot.rgbledCAN.LEDoff();
    //	Robot.rgbledCAN.LEDred();
    	
   	
    	SmartDashboard.putNumber("Auto Finished", DriverStation.getInstance().getMatchTime() );
    	System.out.println("Auto Finished: " + DriverStation.getInstance().getMatchTime());
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
    }
}
