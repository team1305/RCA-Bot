/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Robot;
import frc.robot.subsystems.Subsystem_Drive;


public class Command_ai_loop extends Command {

  private String cstate;
  private double Kp = -0.01f;
  private double Ki = 0.0f;
  private double Kf = 0.05f;  //feedforward - minimum command signal
  private double left_command;
  private double right_command;
  private int izone, irpm;
  private boolean btarget;
  private double zone1threshold = 50; //  pixel width
  public double distance;
  private int iloops = 0;
  

  private double x, thor; //Limelight Values
    
  public Command_ai_loop() {
    requires(Robot.shooter);
    addrequirements(Robot.drive);
    requires(Robot.limelight);
    
    
  } 

  
 
  private void addrequirements(Subsystem_Drive drive) {
}



// Called just before this Command runs the first time.
  @Override
  protected void initialize() {
    cstate = "HUNT";
    izone = 1;
    irpm = 0;
    btarget = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
     // AI Loop, States HUNT, SHOOT

     //When game data isn't blank, and intake is on
     if (Robot.intake.isIntakeOn() && (getGamedata() != ""))  {

      if (iloops <= 50){
        Robot.led.setYellow();
        iloops = iloops +1;
        
      }

      else {
        setColorWheel();
        iloops = iloops +1;

        if (iloops == 100){
          iloops = 0;
        }

      }
    }


      //When game data is blank, and intake is on
      else if (Robot.intake.isIntakeOn() && (getGamedata() == "")){

      Robot.led.setYellow();

      }


     //When target is aquired and game data isn't blank 
     else if (Robot.limelight.is_Target() && (getGamedata() != "")){
      if (iloops <= 50){
        Robot.led.setGreen();
        iloops = iloops +1;
        
      }

      else {
        setColorWheel();
        iloops = iloops +1;

        if (iloops == 100){
          iloops = 0;
        }

      }
      
     }

     //When target is aquired and game data is blank 
      else if (Robot.limelight.is_Target() && (getGamedata() == "")){
      Robot.led.setGreen();
     }

     else {
      Robot.led.setBlack();
     }

    






     //Vision tracking code, activates when right bumper is pressed
     if (Robot.oi.getJoystickDriver().getRawButton(6)) { // Driver RB
        
      //Limelight turn to target
       switch (cstate) {  
       case "HUNT" : 
         // Use Limelight to move to target
         Robot.drive.LowGear();


         thor = Robot.limelight.get_Thor();
         x = Robot.limelight.get_Tx();


         if (Robot.limelight.is_Target()) {



            left_command = Robot.drive.getLeftSide();
            right_command = Robot.drive.getRightSide();
     
            double heading_error = -x;
            double steering_adjust = 0.0f;
                 if (x > 1.5)
                 {
                         steering_adjust = Kp*heading_error - Kf;
                 }
                 else if (x < -1.5)
                 {
                         steering_adjust = Kp*heading_error + Kf;
                 }
     
     
            left_command += steering_adjust;
            right_command -= steering_adjust;
     
            SmartDashboard.putNumber("Left Command", left_command);
            SmartDashboard.putNumber("Right Command", right_command);
            SmartDashboard.putNumber("Steering Adjust", steering_adjust);
            SmartDashboard.putNumber("X", x);
     
     
            if (left_command > 0.75){
              left_command = 0.75;
            }
     
            if (right_command > 0.75){
              right_command = 0.75;
            }
     
            Robot.drive.setLeftSide(-left_command);
            Robot.drive.setRightSide(right_command);
             
            
            if (Math.abs(x) <= 1.5){
               // Calc Distance away so we know zone 1 or zone 2

              
              izone = 1; // default
              /*
              if (thor >= zone1threshold) { // bigger is closer
                izone = 1;
              } else if ( (thor < zone1threshold) && (thor > 0) ) {
                izone = 2;
              } else {
                // should be no target
              }
              */

              distance = Robot.limelight.getDistance();
              if (distance > 0){
                cstate = "SHOOT";
              }
            }
              
         } // if btarget

         //Check hood angle, and shooter speed based on zone
         break;
        case "SHOOT" : 
           if (distance <=60){
             Robot.shooter.hoodUp();
             irpm = 6000;
           }

           else if ((distance > 60) && (distance <= 90)){
             irpm = 6250;
             Robot.shooter.hoodDown();
             
           }

           else if ((distance > 90) && (distance <= 120)){
             irpm = 6500;
             Robot.shooter.hoodDown();
           }

           else{
             irpm = 6750;
             Robot.shooter.hoodDown();
           }

           // Fire up the Shooter
           Robot.shooter.setShooterRPM(irpm);
           
           SmartDashboard.putNumber("shooterRPM", Robot.shooter.getShooterRPM() );
           
           if (Robot.shooter.getShooterRPM() >= irpm) {
              // We are at speed, Turn on feeders 
              Robot.elevator.elevatorUp(0.3);
              Robot.hopper.hopperOut(0.2);
              Robot.intake.enableIntake(0.2);
          }
        break;
      }
      
     } else { // button not pressed
        if (cstate == "SHOOT") {
           // Button no longer pressed but last state was shoot
           // Turn off Shooter and Feeders
           Robot.shooter.setShooterRPM(0);
           Robot.elevator.elevatorUp(0);
           Robot.hopper.hopperOut(0);
           Robot.intake.enableIntake(0);
           
           Robot.shooter.hoodDown();


           Robot.led.setBlack();

           cstate = "HUNT";
           izone = 1;
           irpm = 0;

        } else {
           cstate = "HUNT";
           izone = 1;
           irpm = 0;
        }

     }

  }




  


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }



  // Called once after isFinished returns true
  @Override
  protected void end() {
  

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  public String getGamedata(){
    return DriverStation.getInstance().getGameSpecificMessage();
  }


  public void setColorWheel(){
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
        Robot.led.setBlue();
          break;
        case 'G' :
        Robot.led.setGreen();
          break;
        case 'R' :
        Robot.led.setRed();
          break;
        case 'Y' :
        Robot.led.setYellow();
          break;
        default :
        Robot.led.setBlack();
          break;
      }
    } else {
      //Code for no data received yet
    }
  }
  
}
