package frc.robot.auto_commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 *
 */
public class NewAutoDriveReverse extends Command {
	

	double distance;
	double AnglePowerFactor;
	double RampUpDist;
	double RampDownDist;
	double MinSpeed;
	double angle;
	double power;
	double currPos;


    public NewAutoDriveReverse(double angle, double distance,  double power, double minpower,double rampup, double rampdown) {
    	
    	
    	
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drive);
        
        this.distance = distance * 147.083;
        this.angle = angle;
        this.power = power;
        
        
        MinSpeed = minpower; //set to just enough power to move bot
        AnglePowerFactor = .3; /// 0.1 = 10%
        RampUpDist = rampup; 
        RampDownDist = rampdown; 
        
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	currPos = 0;
    	//Robot.drive.resetEncoder();
    	//RobotMap.driveCANTalonLeftFront.reset();

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	currPos =  Robot.drive.getDistance() ; 	
    	
    	double dif = angle - Robot.drive.gyroGetAngle();

    	    	
    	if (currPos  < RampUpDist ) {  //ramp up
    		double RampUpPercent = (currPos /RampUpDist);
    		double SetPower = (MinSpeed + ((power - MinSpeed)  * RampUpPercent)); 
    		
    		double speedleft = SetPower + dif * AnglePowerFactor; //add or subtract power to left
    		double speedright = SetPower - dif * AnglePowerFactor; //add or subtract power to right
    		
    		
//    		speedleft = Math.min(speedleft, power);
//    		speedright = Math.min(speedright, power);
    		
    		if (speedleft < MinSpeed) {  //required if angle difference is large so we do not get negative speeds
    			speedleft = Math.max(speedleft, power);
    		}else { 
    			speedleft = MinSpeed;
    		}
    		
    		if (speedright < MinSpeed) {
    			speedright = Math.max(speedright, power);
    		}else { 
    			speedright = MinSpeed;
    		}

    		Robot.drive.driveTank(speedleft, speedright);
    		
    		
    	}else if ((distance - currPos) < RampDownDist ){ //ramp down
    		double RampDownPercent =  (( distance - currPos ) / RampDownDist) ;
    		double SetPower = MinSpeed + ((power - MinSpeed)  * RampDownPercent); 
    		
  		
    		double speedleft = SetPower + dif * AnglePowerFactor; //add or subtract power to left
    		double speedright = SetPower - dif * AnglePowerFactor; //add or subtract power to right

    		
    		if (speedleft < MinSpeed) {  //required if angle difference is large so we do not get negative speeds
    			speedleft = Math.max(speedleft, power);
    		}else { 
    			speedleft = MinSpeed;
    		}
    		
    		if (speedright < MinSpeed) {
    			speedright = Math.max(speedright, power);
    		}else { 
    			speedright = MinSpeed;
    		}
    		
    		Robot.drive.driveTank(speedleft, speedright);
			
    	}else { 
    		
    		double SetPower = power;
    		
    		double speedleft = SetPower + dif * AnglePowerFactor; //add or subtract power to left
    		double speedright = SetPower - dif * AnglePowerFactor; //add or subtract power to right
    		
	    		if (speedleft < MinSpeed) {  //required if angle difference is large so we do not get negative speeds
	    			speedleft = Math.max(speedleft, power);
	    		}else { 
	    			speedleft = MinSpeed;
	    		}
	    		
	    		if (speedright < MinSpeed) {
	    			speedright = Math.max(speedright, power);
	    		}else { 
	    			speedright = MinSpeed;
	    		}

    		Robot.drive.driveTank(speedleft, speedright);
    		
        }
    	
    	SmartDashboard.putNumber("currAngle", Robot.drive.gyroGetAngle());
    	SmartDashboard.putNumber("currPos", Robot.drive.getDistance());
    	SmartDashboard.putNumber("SetDistance", distance);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	//return true;
    	//return (Math.abs(distance - currPos) <= .01);
    	//return ((Math.abs(currPos) - Math.abs(distance) <= 0.1);
    	//return (Math.abs(currPos) - (Math.abs(distance)) <= 0.1);
    	return (currPos <= distance); 
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
