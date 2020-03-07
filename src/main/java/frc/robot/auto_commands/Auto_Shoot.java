package frc.robot.auto_commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 *
 */
public class Auto_Shoot extends Command {
    double distance;
    double irpm;
    int loopCounter;
    int seconds;

    public Auto_Shoot(int seconds) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        this.seconds = seconds;

        requires(Robot.shooter);
        loopCounter = 0;

    }

protected void initialize() {
    setTimeout(6);
    getDistanceAuto();
}

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        // Fire up the Shooter
        Robot.shooter.setShooterRPM(irpm);
        SmartDashboard.putNumber("irpm_auto", irpm);

        SmartDashboard.putNumber("shooterRPM", Robot.shooter.getShooterRPM());
        SmartDashboard.putNumber("thedistance", distance);
        double droppedIrpm = irpm - 50;

        if (Robot.shooter.getShooterRPM() >= droppedIrpm) {
            // We are at speed, Turn on feeders
            Robot.elevator.elevatorUp(0.5);
            Robot.hopper.hopperOut(0.4);
            Robot.intake.enableIntake(0.4);
            loopCounter = loopCounter + 1;
        }

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (isTimedOut()) {
            return true;
        } else {
           if (loopCounter >= (seconds*20)) {
               return true;

           } else {
               return false;
           }
        }
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.shooter.setShooterSpeed(0);
        Robot.elevator.elevatorUp(0);
        Robot.hopper.hopperOut(0);
        Robot.intake.enableIntake(0);

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }

    private void getDistanceAuto(){
        distance = Robot.limelight.getDistance();

        if (distance <= 200) { // 1205
            Robot.shooter.hoodDown();
            irpm = 4000;
            Robot.shooter.setShooterPIDInfrontOfLine();

        }

        else if ((distance > 200) && (distance <= 300)) { // 120, 259
            irpm = 4000;

            Robot.shooter.hoodDown();
            Robot.shooter.setShooterPIDInitiationLine();

        }

        else if ((distance > 300) && (distance <= 350)) {// 259, 450
            irpm = 5000;
            Robot.shooter.hoodUp();
            Robot.shooter.setShooterPIDTrench();
        }

        else if ((distance > 350) && (distance <= 400)) {// 259, 450
            irpm = 5500;
            Robot.shooter.hoodUp();
            Robot.shooter.setShooterPIDTrench();
        }

        else if ((distance > 400) && (distance <= 500)) {// 259, 450
            irpm = 5750;
            Robot.shooter.hoodUp();
            Robot.shooter.setShooterPIDTrench();
        }

        else if ((distance > 500) && (distance <= 600)){//259, 450
            irpm = 5750;
            Robot.shooter.hoodDown();
            Robot.shooter.setShooterPIDTrench();
          }

        else {
            irpm = 6000;
            Robot.shooter.hoodUp();
            Robot.shooter.setShooterPIDTrenchBack();
        }
      
    }
}
