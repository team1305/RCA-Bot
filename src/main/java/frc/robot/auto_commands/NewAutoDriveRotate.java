package frc.robot.auto_commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 *
 */
public class NewAutoDriveRotate extends Command {
	
	

    
	private double SetAngle;
	private double SetMaxSpeed;
	private double RampDownAngle;
	private double MinSpeed;
	


    public NewAutoDriveRotate(double angle, double power, double minpower, double rampdown) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drive);
        
    
        this.SetAngle = angle;
        this.SetMaxSpeed = power;
        
        MinSpeed = minpower; //set to just enough power to move bot
        RampDownAngle = rampdown; 
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	double CurrentAngle = Robot.drive.gyroGetAngle();
    	double AngleDifAbs = Math.abs(SetAngle - CurrentAngle);
    	   	    	
    	
    		double RampUpPercent = (AngleDifAbs / RampDownAngle);
    		double SetPower = (MinSpeed + ((SetMaxSpeed - MinSpeed)  * RampUpPercent)); 
    		SetPower = Math.min(SetPower, SetMaxSpeed);
    		
    		if (CurrentAngle > SetAngle){
				Robot.drive.driveTank(-SetPower, SetPower);
			}else {
				Robot.drive.driveTank(SetPower, -SetPower);
   	        }
    	
    	SmartDashboard.putNumber("GyroAngle", Robot.drive.gyroGetAngle());
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        //return true;
    	///return Math.abs( Robot.drive.gyroGetAngle() - SetAngle)  < 1;
    	///return  (SetAngle - Robot.drive.gyroGetAngle() )  > 0;
    	if (SetAngle <= 0) {  	
    		return  Robot.drive.gyroGetAngle() <= SetAngle ;
    	}else{
    		return  Robot.drive.gyroGetAngle() >= SetAngle ;
        }
        
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
