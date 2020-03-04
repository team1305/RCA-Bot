package frc.robot.auto_commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 *
 */
public class Auto_Drive_Rev extends Command {

	double distance1;
	double AnglePowerFactor;
	double RampUpDist;
	double RampDownDist;
	double MinSpeed;
	double angle1;
	double power;
	double currPos;
   
	
    public Auto_Drive_Rev(double angle1, double distance1, double power, double minpower, double rampup ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	 requires(Robot.drive);
         
         this.distance1 = -distance1 * Robot.drive.getratio_high(); 
         this.angle1 = angle1;
         this.power = -power;
        
         
         
         MinSpeed = -minpower; //set to just enough power to move bot
         AnglePowerFactor = .1; /// 0.1 = 10%
         RampUpDist = -rampup * Robot.drive.getratio_high();      	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	currPos = -.1;
    	Robot.drive.resetEncoder();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    	currPos =  Robot.drive.getDistance() ; 	
    	
    	double dif = angle1;// - Robot.navX.getYaw();

    	SmartDashboard.putNumber("Average Drive Encoder", Robot.drive.getDistance());
    	SmartDashboard.putNumber("Right Drive Encoder", Robot.drive.getDistance_Right());
    	SmartDashboard.putNumber("Left Drive Encoder", Robot.drive.getDistance_Left());

    	//SmartDashboard.putNumber("NavX getYaw", Robot.navX.getYaw());
    	
    	if (currPos  < RampUpDist ) {  //ramp up
    		double RampUpPercent = (currPos /RampUpDist);
    		double SetPower = (MinSpeed + ((power - MinSpeed)  * RampUpPercent)); 
    		
    		double speedleft = SetPower - dif * AnglePowerFactor; //add or subtract power to left
    		double speedright = SetPower + dif * AnglePowerFactor; //add or subtract power to right
    		
    		
    		if (speedleft > MinSpeed) {  //required if angle difference is large so we do not get negative speeds
    			speedleft = Math.min(speedleft, power);
    		}else { 
    			speedleft = MinSpeed;
    		}
    		
    		if (speedright > MinSpeed) {
    			speedright = Math.min(speedright, power);
    		}else { 
    			speedright = MinSpeed;
    		}

    		Robot.drive.driveTank(speedleft, speedright);
    		
    		
    	} else { 	// Loop for Distance 1
    		double SetPower = power;
    		
    		double speedleft = SetPower - dif * AnglePowerFactor; //add or subtract power to left
    		double speedright = SetPower + dif * AnglePowerFactor; //add or subtract power to right
    		
	    		if (speedleft > MinSpeed) {  //required if angle difference is large so we do not get negative speeds
	    			speedleft = Math.min(speedleft, power);
	    		}else { 
	    			speedleft = MinSpeed;
	    		}
	    		
	    		if (speedright > MinSpeed) {
	    			speedright = Math.min(speedright, power);
	    		}else { 
	    			speedright = MinSpeed;
	    		}

    		Robot.drive.driveTank(speedleft, speedright);
    		
        }
    	
//    	SmartDashboard.putNumber("currAngle", Robot.drive.gyroGetAngle());
//    	SmartDashboard.putNumber("currPos", Robot.drive.getDistance());
//    	SmartDashboard.putNumber("SetDistance", distance);  
      // 	SmartDashboard.putNumber("NavX getYaw", Robot.navX.getYaw());
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Math.abs(distance1) < Math.abs(currPos);
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
