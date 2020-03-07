package frc.robot.auto_commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 *
 */
public class Auto_Drive_Master extends Command {

	double distance1;
	double AnglePowerFactor;
	double RampUpDist;
	double RampDownDist;
	double MinSpeed;
	double angle1;
	double power;
	double currPos;
	double startPos;
	double target;

	double dif;

	double RampUpPercent;
	double SetPower;

	double speedleft;
	double speedright;


	public Auto_Drive_Master(double angle1, double distance1, double power, double minpower, double rampup,
			double rampdown) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);

		// requires(Robot.drive);

		this.distance1 = distance1 * Robot.drive.getratio_high(); // converts distance to encoder values
		this.angle1 = angle1;
		this.power = power;

		MinSpeed = minpower; // set to just enough power to move bot
		AnglePowerFactor = .05; /// 0.1 = 10%
		RampUpDist = rampup * Robot.drive.getratio_high();
		RampDownDist = rampdown * Robot.drive.getratio_high();
		
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		currPos = -.1;
		//Robot.drive.resetEncoder();
		startPos = Robot.drive.getDistance();

		target = startPos + distance1;

		// Robot.drive.LowGear();
		SmartDashboard.putString("Starting Auto Drive", "yes");
		SmartDashboard.putNumber("Auto Drive Start", startPos);
		SmartDashboard.putNumber("Target Auto Drive", target);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		currPos = Robot.drive.getDistance();

		if (target > startPos) { // go forward

			if (currPos <= (startPos + RampUpDist)) {
				dif = angle1 - Robot.drive.gyroGetAngle();
				//dif = 0;

				RampUpPercent = (currPos / (startPos + RampUpDist));
				SetPower = (MinSpeed + ((power - MinSpeed) * RampUpPercent));

				speedleft = SetPower + (dif * AnglePowerFactor); // add or subtract power to left
				speedright = SetPower - (dif * AnglePowerFactor); // add or subtract power to right

				if (speedleft > MinSpeed) { // required if angle difference is large so we do not get negative speeds
					speedleft = Math.min(speedleft, power);
				} else {
					speedleft = MinSpeed;
				}

				if (speedright > MinSpeed) {
					speedright = Math.min(speedright, power);
				} else {
					speedright = MinSpeed;
				} // end of ramp up

			} else if (currPos >= (target - RampDownDist)) {
				dif = angle1 - Robot.drive.gyroGetAngle();
				//dif = 0;

				RampUpPercent = (currPos / (target - RampDownDist));
				SetPower = (MinSpeed + ((power - MinSpeed) * RampUpPercent));

				speedleft = SetPower + (dif * AnglePowerFactor); // add or subtract power to left
				speedright = SetPower - (dif * AnglePowerFactor); // add or subtract power to right

				if (speedleft > MinSpeed) { // required if angle difference is large so we do not get negative speeds
					speedleft = Math.min(speedleft, power);
				} else {
					speedleft = MinSpeed;
				}

				if (speedright > MinSpeed) {
					speedright = Math.min(speedright, power);
				} else {
					speedright = MinSpeed;
				} // end of ramp up

			} else {
				// drive full speed * direction;
				SetPower = power;
				dif = angle1 - Robot.drive.gyroGetAngle();
				//dif = 0;

				speedleft = SetPower + (dif * AnglePowerFactor); // add or subtract power to left
				speedright = SetPower - (dif * AnglePowerFactor); // add or subtract power to right

				if (speedleft > MinSpeed) { // required if angle difference is large so we do not get negative speeds
					speedleft = Math.min(speedleft, power);
				} else {
					speedleft = MinSpeed;
				}

				if (speedright > MinSpeed) {
					speedright = Math.min(speedright, power);
				} else {
					speedright = MinSpeed;
				}

			}

		   Robot.drive.setRightSide(speedright);
		   Robot.drive.setLeftSide(-speedleft);


		} else { // go backwards
			if ((currPos <= startPos) && (currPos >= (startPos - RampUpDist))) {
				dif = angle1 + Robot.drive.gyroGetAngle();
				//dif = 0;

				RampUpPercent = (currPos / (startPos - RampUpDist));
				SetPower = (MinSpeed + ((power - MinSpeed) * RampUpPercent));

				speedleft = SetPower + (dif * AnglePowerFactor); // add or subtract power to left
				speedright = SetPower - (dif * AnglePowerFactor); // add or subtract power to right

				if (speedleft > MinSpeed) { // required if angle difference is large so we do not get negative speeds
					speedleft = Math.min(speedleft, power);
				} else {
					speedleft = MinSpeed;
				}

				if (speedright > MinSpeed) {
					speedright = Math.min(speedright, power);
				} else {
					speedright = MinSpeed;
				} // end of ramp up


			} else if (currPos <= (target + RampDownDist)) {
				dif = angle1 +  Robot.drive.gyroGetAngle();
				//dif = 0;

				RampUpPercent = (currPos / (target + RampDownDist));
				SetPower = (MinSpeed + ((power - MinSpeed) * RampUpPercent));

				speedleft = SetPower + (dif * AnglePowerFactor); // add or subtract power to left
				speedright = SetPower - (dif * AnglePowerFactor); // add or subtract power to right

				if (speedleft > MinSpeed) { // required if angle difference is large so we do not get negative speeds
					speedleft = Math.min(speedleft, power);
				} else {
					speedleft = MinSpeed;
				}

				if (speedright > MinSpeed) {
					speedright = Math.min(speedright, power);
				} else {
					speedright = MinSpeed;
				} // end of ramp up


			} else {
				// drive full speed * direction;
				SetPower = power;
				dif = angle1 +  Robot.drive.gyroGetAngle();
				//dif = 0;

				speedleft = SetPower + (dif * AnglePowerFactor); // add or subtract power to left
				speedright = SetPower - (dif * AnglePowerFactor); // add or subtract power to right

				if (speedleft > MinSpeed) { // required if angle difference is large so we do not get negative speeds
					speedleft = Math.min(speedleft, power);
				} else {
					speedleft = MinSpeed;
				}

				if (speedright > MinSpeed) {
					speedright = Math.min(speedright, power);
				} else {
					speedright = MinSpeed;
				}
			}
			
			SmartDashboard.putNumber("speedleft", speedleft);
			SmartDashboard.putNumber("speedright", -speedright);
			
			Robot.drive.setRightSide(-speedright);
			Robot.drive.setLeftSide(speedleft);
 
		}
/*
		// old code
		currPos = Robot.drive.getDistance();
		SmartDashboard.putNumber("currPos", currPos);

		double dif = angle1;// - Robot.navX.getYaw();

		// SmartDashboard.putNumber("NavX getYaw", Robot.navX.getYaw());

		if (currPos < RampUpDist) { // ramp up
			double RampUpPercent = (currPos / (startPos + RampUpDist));
			double SetPower = (MinSpeed + ((power - MinSpeed) * RampUpPercent));

			double speedleft = SetPower + dif * AnglePowerFactor; // add or subtract power to left
			double speedright = SetPower - dif * AnglePowerFactor; // add or subtract power to right

			if (speedleft > MinSpeed) { // required if angle difference is large so we do not get negative speeds
				speedleft = Math.min(speedleft, power);
			} else {
				speedleft = MinSpeed;
			}

			if (speedright > MinSpeed) {
				speedright = Math.min(speedright, power);
			} else {
				speedright = MinSpeed;
			}

			Robot.drive.setRightSide(speedright);
			Robot.drive.setLeftSide(-speedleft);
			// Robot.drive.driveTank(speedleft, speedright);
			SmartDashboard.putNumber("Ramp up distance", RampUpDist);
			SmartDashboard.putNumber("speedleft", speedleft);
			SmartDashboard.putNumber("speedright", speedright);

		} else { // Loop for Distance 1
			double SetPower = power;

			double speedleft = SetPower + dif * AnglePowerFactor; // add or subtract power to left
			double speedright = SetPower - dif * AnglePowerFactor; // add or subtract power to right

			if (speedleft > MinSpeed) { // required if angle difference is large so we do not get negative speeds
				speedleft = Math.min(speedleft, power);
			} else {
				speedleft = MinSpeed;
			}

			if (speedright > MinSpeed) {
				speedright = Math.min(speedright, power);
			} else {
				speedright = MinSpeed;
			}

			Robot.drive.setRightSide(speedright);
			Robot.drive.setLeftSide(-speedleft);
			// Robot.drive.driveTank(speedleft, speedright);

		}

		// SmartDashboard.putNumber("currAngle", Robot.drive.gyroGetAngle());
		// SmartDashboard.putNumber("currPos", Robot.drive.getDistance());
		// SmartDashboard.putNumber("SetDistance", distance);
		// SmartDashboard.putNumber("NavX getYaw", Robot.navX.getYaw());
*/
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		SmartDashboard.putNumber("currPos", currPos);
		SmartDashboard.putNumber("target", target);


		if (target >= startPos) { // going forward
			return (currPos >= target);

		} else {
			return (currPos <= target);
		}
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
