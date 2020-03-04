package frc.robot.auto_commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Subsystem_Drive;

/**
 *
 */
public class Auto_Drive_Curve extends Command {
	double distance;
	double AnglePowerFactor;
	double RampUpDist;
	double RampDownDist;
	double MinSpeed;
	double angle;
	double power;
	double currPos;
	double startpos;

	public Auto_Drive_Curve(double angle, double radius, double power, double minpower, double rampup,
			double rampdown) {

		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Robot.drive);
		startpos = Subsystem_Drive.getDistance();
		// distance = ((2)*(Math.PI)*(radius))/4;

		this.distance = radius * Robot.drive.getratio_high();
		this.angle = angle;
		this.power = power;

		MinSpeed = minpower; // set to just enough power to move bot
		AnglePowerFactor = .1; /// 0.1 = 10%
		RampUpDist = rampup * Robot.drive.getratio_high();
		RampDownDist = rampdown * Robot.drive.getratio_high();

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		currPos = 0;
		Robot.drive.resetEncoder();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		currPos = Robot.drive.getDistance_Right();

		double NewAngleTarget = (currPos / distance * angle);
		double dif = NewAngleTarget - Robot.drive.gyroGetAngle();

		if (currPos < RampUpDist) { // ramp up
			double RampUpPercent = (currPos / RampUpDist);
			double SetPower = (MinSpeed + ((power - MinSpeed) * RampUpPercent));

			double speedleft = SetPower + dif * AnglePowerFactor; // add or subtract power to left
			double speedright = SetPower - dif * AnglePowerFactor; // add or subtract power to right

			speedleft = Math.min(speedleft, power);
			speedright = Math.min(speedright, power);

			Robot.drive.setRightSide(speedright);
			Robot.drive.setLeftSide(-speedleft);

			// Robot.drive.driveTank(speedleft, speedright);

		} else if ((distance - currPos) < RampDownDist) { // ramp down
			double RampDownPercent = ((distance - currPos) / RampDownDist);
			double SetPower = MinSpeed + ((power - MinSpeed) * RampDownPercent);

			double speedleft = SetPower + dif * AnglePowerFactor; // add or subtract power to left
			double speedright = SetPower - dif * AnglePowerFactor; // add or subtract power to right

			speedleft = Math.min(speedleft, power);
			speedright = Math.min(speedright, power);

			Robot.drive.setRightSide(speedright);
			Robot.drive.setLeftSide(-speedleft);

		} else {

			double SetPower = power;

			double speedleft = SetPower + dif * AnglePowerFactor; // add or subtract power to left
			double speedright = SetPower - dif * AnglePowerFactor; // add or subtract power to right

			speedleft = Math.min(speedleft, power);
			speedright = Math.min(speedright, power);

			Robot.drive.setRightSide(speedright);
			Robot.drive.setLeftSide(-speedleft);

		}

		SmartDashboard.putNumber("currAngleCurve", Robot.drive.gyroGetAngle());
		SmartDashboard.putNumber("currPosCurve", Subsystem_Drive.getDistance());
    	SmartDashboard.putNumber("SetDistanceCurve", distance);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {

		/*if (distance < 0){
			return distance > currPos;

		}
		else{
			return distance < currPos;
		}*/

    	//return (( DistTravelled - SetDist  )  > 0);
		   //return (Math.abs(distance - currPos) <= .01);
		   
		   
       	return ((Math.abs(distance) - Math.abs(currPos)) <= .01);
    	
    	
    }

    // Called once after isFinished returns true
    protected void end() {
		Robot.drive.resetEncoder();
    	    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
