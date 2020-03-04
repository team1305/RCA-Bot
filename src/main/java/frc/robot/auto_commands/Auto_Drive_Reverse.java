package frc.robot.auto_commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 *
 */
public class Auto_Drive_Reverse extends Command {

	double distance1;
	double AnglePowerFactor;
	double RampUpDist;
	double RampDownDist;
	double MinSpeed;
	double angle1;
	double power;
	double currPos;
	double startpos;
   
	
    public Auto_Drive_Reverse(double angle1, double distance1, double power, double minpower, double rampup) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	 //requires(Robot.drive);
         
         this.distance1 = distance1 * Robot.drive.getratio_high(); // converts distance to encoder values
         this.angle1 = angle1;
         this.power = power;
        
         //startpos = Robot.drive.getDistance();
		 startpos = 0;
		 
         MinSpeed = minpower; //set to just enough power to move bot
         AnglePowerFactor = .1; /// 0.1 = 10%
         RampUpDist = rampup * Robot.drive.getratio_high();;      	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	currPos = -.1;
		Robot.drive.resetEncoder();
		//Robot.drive.LowGear();
		SmartDashboard.putString("Staring Auto Drive", "yes");
		SmartDashboard.putNumber("Target Auto Drive", distance1);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		

		currPos =  Robot.drive.getDistance() ; 	
		SmartDashboard.putNumber("currPos", currPos);
    	
    	double dif = angle1;// - Robot.navX.getYaw();

    	
    //	SmartDashboard.putNumber("NavX getYaw", Robot.navX.getYaw());
    	
    	if (currPos  < (startpos - RampUpDist)) {  //ramp up
    		double RampUpPercent = (currPos /(startpos - RampUpDist));
    		double SetPower = (MinSpeed + ((power - MinSpeed)  * RampUpPercent)); 
    		
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
			
			
			Robot.drive.setRightSide(-speedright);
			Robot.drive.setLeftSide(speedleft);
			//Robot.drive.driveTank(speedleft, speedright);
			SmartDashboard.putNumber("Ramp up distance", RampUpDist);
			SmartDashboard.putNumber("speedleft", speedleft);
			SmartDashboard.putNumber("speedright", speedright);
    		
    		
    	} else { 	// Loop for Distance 1
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

				Robot.drive.setRightSide(-speedright);
				Robot.drive.setLeftSide(speedleft);
    		//Robot.drive.driveTank(speedleft, speedright);
    		
        }
    	
//    	SmartDashboard.putNumber("currAngle", Robot.drive.gyroGetAngle());
//    	SmartDashboard.putNumber("currPos", Robot.drive.getDistance());
//    	SmartDashboard.putNumber("SetDistance", distance);  
       //	SmartDashboard.putNumber("NavX getYaw", Robot.navX.getYaw());
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
		
	return (startpos + distance1) < currPos;


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
