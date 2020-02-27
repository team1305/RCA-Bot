package frc.robot.auto_commands;


import com.ctre.phoenix.motorcontrol.MotorCommutation;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 *
 */
public class NewAutoDriveStraight extends Command {
	

	double distance1;
	double AnglePowerFactor;
	double RampUpDist;
	double RampDownDist;
	double MinSpeed;
	double angle1;
	double power;
	double currPos;
    double angle2, distance2;

    //public NewAutoDriveStraight(double angle, double distance,  double power, double minpower,double rampup, ) {
    	
    public NewAutoDriveStraight(double angle1, double distance1, double angle2, double distance2,  double power, double minpower, double rampup ) {
    	    	
    	
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drive);
		
		//2048 = 1 rotation of Motor
		// Multiple by Gear Rotation
		

        this.distance1 = distance1 * 147.083; // What is this years multiplier?
        this.angle1 = angle1;
        this.power = power;
        this.angle2 = angle2;
        this.distance2 = distance2 * 147.083;
        
        
        MinSpeed = minpower; //set to just enough power to move bot
        AnglePowerFactor = .1; /// 0.1 = 10%
        RampUpDist = rampup * 147.083; 
      //  RampDownDist = rampdown; 
        
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	currPos = -.1;
    	//Robot.drive.resetEncoder();
    	//RobotMap.driveCANTalonLeftFront.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	currPos =  Robot.drive.getDistance() ; 	
    	
    	double dif = angle1 - Robot.drive.gyroGetAngle();

    	
    	
    	if (currPos  < RampUpDist ) {  //ramp up
    		double RampUpPercent = (currPos /RampUpDist);
    		double SetPower = (MinSpeed + ((power - MinSpeed)  * RampUpPercent)); 
    		
    		double speedleft = SetPower + dif * AnglePowerFactor; //add or subtract power to left
    		double speedright = SetPower - dif * AnglePowerFactor; //add or subtract power to right
    		
    		
//    		speedleft = Math.min(speedleft, power);
//    		speedright = Math.min(speedright, power);
    		
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

    		Robot.drive.driveTank(speedright, speedleft);
    		
    		
    	}else if (currPos > distance1) { //do curve
    	//	double RampDownPercent =  (( distance - currPos ) / RampDownDist) ;
    		double SetPower = power; 
    		
    		double NewAngleTarget = ( (currPos - distance1)/distance2) * angle2 ;
        	double dif2 = NewAngleTarget - Robot.drive.gyroGetAngle();
        	
    		double speedleft = SetPower - 0.1 + dif2 * 0.05; //AnglePowerFactor; //add or subtract power to left
    		double speedright = SetPower - 0.1 - dif2 * 0.05; //AnglePowerFactor; //add or subtract power to right
     		
//    		speedleft = Math.min(speedleft, power);
//    		speedright = Math.min(speedright,power );
    		Robot.drive.driveTank(speedright, speedleft);
    		
    		
    	}else { 
    		
    		double SetPower = power;
    		
    		double speedleft = SetPower + dif * AnglePowerFactor; //add or subtract power to left
    		double speedright = SetPower - dif * AnglePowerFactor; //add or subtract power to right
    		
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

    		Robot.drive.driveTank(speedright, speedleft);
    		
        }
    	
//    	SmartDashboard.putNumber("currAngle", Robot.drive.gyroGetAngle());
//    	SmartDashboard.putNumber("currPos", Robot.drive.getDistance());
//    	SmartDashboard.putNumber("SetDistance", distance);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	//return true;
    	//return (Math.abs(distance - currPos) <= .01);
    	return (Math.abs(distance1) + Math.abs(distance2)) < Math.abs(currPos);
    	
    	
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
