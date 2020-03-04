package frc.robot.auto_commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 *
 */
public class Auto_Drive_Gyro extends Command {

	double distance1;
	double AnglePowerFactor;
	double RampUpDist;
	double RampDownDist;
	double MinSpeed;
	double angle1;
	double power;
	double currPos;
	double heading;
	double kP = 1;
   
	
    public Auto_Drive_Gyro(double angle1, double distance1, double power, double minpower, double rampup) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	 requires(Robot.drive);
     	
    }

    // Called just before this Command runs the first time
    protected void initialize() {

            		//heading = 90 - Robot.drive.gyroGetAngle();
		double error = 90 - Robot.drive.gyroGetAngle();

		// Drives forward continuously at half speed, using the gyro to stabilize the heading
    Robot.drive.DriveTank(kP * error, kP * error);
    
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {


    

      SmartDashboard.putNumber("Gyro Angle", Robot.drive.gyroGetAngle());
		
    	
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
			return true;
	
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.DriveStop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
