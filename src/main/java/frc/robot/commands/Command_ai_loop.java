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
  
  private double Kp = -0.03f;
  private double Ki = 0.012f; // 0.006
  private double Kf = 0.05f;  //feedforward - minimum command signal
  
  private double left_command;
  private double right_command;
  
  private double x;
  private int izone, irpm;
  private boolean btarget;
  private double zone1threshold = 50; //  pixel width
  public double distance;
  private int iloops = 0;
  
  private int isuccess = 0; 

    
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
    isuccess = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
     // AI Loop, States HUNT, SHOOT

     // Colour Loop
     set_colors();

     //Vision tracking code for distance
     if (Robot.oi.getJoystickDriver().getRawButton(5)){ //Driver LB
      switch (cstate){
        case "HUNT":
          Robot.drive.LowGear();
          Robot.shooter.setShooterRPM(4000);
          Robot.limelight.trackToDistance(0);//Offset from target
          if (Math.abs(Robot.limelight.get_Tx()) <= 1 ){
            if (Math.abs(Robot.limelight.get_Ty()) <=1){
              cstate = "SHOOT";
            }
          }
          break;


        case "SHOOT":
         distance = Robot.limelight.getDistance();

         if (distance <=200){ //1205
          Robot.shooter.hoodDown();
          irpm = 4000;
          Robot.shooter.setShooterPIDInfrontOfLine();

          }
        else{

        }

        Robot.shooter.setShooterRPM(irpm);
        SmartDashboard.putNumber("irpm", irpm);
           
        SmartDashboard.putNumber("shooterRPM", Robot.shooter.getShooterRPM() );
        SmartDashboard.putNumber("thedistance", distance );
        double droppedIrpm = irpm - 50;

        if (Robot.shooter.getShooterRPM() >= droppedIrpm) {
              // We are at speed, Turn on feeders 
              Robot.elevator.elevatorUp(0.5);
              Robot.hopper.hopperOut(0.4);
              Robot.intake.enableIntake(0.4);
        }


      }
     }

      else { // button not pressed
        if (cstate == "SHOOT") { // RECOVERY STATE
           // Button no longer pressed but last state was shoot
           // Turn off Shooter and Feeders
           Robot.shooter.setShooterRPM(0);
           Robot.elevator.elevatorUp(0);
           Robot.hopper.hopperOut(0);
           Robot.intake.enableIntake(0);
           Robot.shooter.hoodDown();
           Robot.drive.HighGear();

           Robot.led.setBlack();

           cstate = "HUNT";
           izone = 1;
           irpm = 0;

        } 
        else { // Reset back to HUNT State
           cstate = "HUNT";
           izone = 1;
           irpm = 0;
        }

     }



     


     //Vision tracking code, activates when right bumper is pressed
     if (Robot.oi.getJoystickDriver().getRawButton(6)) { // Driver RB

       //Limelight turn to target
       switch (cstate) {  
       case "HUNT" : 
         // Use Limelight to move to target
         Robot.drive.LowGear();
         Robot.shooter.setShooterRPM(4000); // ramp up to 4000 rpm as base so shoot will be faster

         // display target distance when in hunt state
         //SmartDashboard.putNumber("thedistance", Robot.limelight.getDistance());
         x = Robot.limelight.get_Tx();

         Robot.limelight.turnRobotToAnlge(x);

         if (Math.abs(x) <= 1.5){
          // Calc Distance away so we know zone 1 or zone 2

          isuccess = isuccess + 1;
         
          izone = 1; // default
        
          distance = Robot.limelight.getDistance();
          SmartDashboard.putNumber("thedistance", distance );

          if (distance > 0){
            if (isuccess >= 5) {
            cstate = "SHOOT";
           
            }
           SmartDashboard.putNumber("isuccess", isuccess);
          }
        }

         /*
         x = Robot.limelight.get_Tx();
        
         if (Robot.limelight.is_Target()) {
          left_command = Robot.drive.getLeftSide();
          right_command = Robot.drive.getRightSide();
    
          double heading_error = -x;
          double steering_adjust = 0.0f;
               if (x > 1.5)
               {
                       steering_adjust = Kp*heading_error + Kf;
               }
               else if (x < -1.5)
               {
                       steering_adjust = Kp*heading_error - Kf;
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

               isuccess = isuccess + 1;
              
              izone = 1; // default
             
              distance = Robot.limelight.getDistance();
              SmartDashboard.putNumber("thedistance", distance );

              if (distance > 0){
                if (isuccess >= 5) {
                cstate = "SHOOT";
                
                }
                SmartDashboard.putNumber("isuccess", isuccess);
              }
            }
              
         } // if btarget
         */

         //Check hood angle, and shooter speed based on zone
         break;
        case "SHOOT" : 
           

           isuccess = 0;

           if (distance <=200){ //1205
             Robot.shooter.hoodDown();
             irpm = 4000;
             Robot.shooter.setShooterPIDInfrontOfLine();

           }

           else if ((distance > 200) && (distance <= 300)){ //120, 259
             irpm = 4000;

             Robot.shooter.hoodDown();
             Robot.shooter.setShooterPIDInitiationLine();
             
           }

           else if ((distance > 300) && (distance <= 350)){//259, 450
             irpm = 5000;
             Robot.shooter.hoodUp();
             Robot.shooter.setShooterPIDTrench();
           }

           else if ((distance > 350) && (distance <= 400)){//259, 450
            irpm = 5500;
            Robot.shooter.hoodUp();
            Robot.shooter.setShooterPIDTrench();
          }

           else if ((distance > 400) && (distance <= 500)){//259, 450
            irpm = 5750;
            Robot.shooter.hoodUp();
            Robot.shooter.setShooterPIDTrench();
          }



           else{
             irpm = 6000;
             Robot.shooter.hoodUp();
             Robot.shooter.setShooterPIDTrenchBack();
           }

           // Fire up the Shooter
           Robot.shooter.setShooterRPM(irpm);
           SmartDashboard.putNumber("irpm", irpm);
           
           SmartDashboard.putNumber("shooterRPM", Robot.shooter.getShooterRPM() );
           SmartDashboard.putNumber("thedistance", distance );
           double droppedIrpm = irpm - 50;

           if (Robot.shooter.getShooterRPM() >= droppedIrpm) {
              // We are at speed, Turn on feeders 
              Robot.elevator.elevatorUp(0.5);
              Robot.hopper.hopperOut(0.4);
              Robot.intake.enableIntake(0.4);
          }
        break;
      }
      
     } 
     else { // button not pressed
        if (cstate == "SHOOT") { // RECOVERY STATE
           // Button no longer pressed but last state was shoot
           // Turn off Shooter and Feeders
           Robot.shooter.setShooterRPM(0);
           Robot.elevator.elevatorUp(0);
           Robot.hopper.hopperOut(0);
           Robot.intake.enableIntake(0);
           
           Robot.shooter.hoodDown();

           Robot.drive.HighGear();

           Robot.led.setBlack();

           cstate = "HUNT";
           izone = 1;
           irpm = 0;

        } else { // Reset back to HUNT State
           cstate = "HUNT";
           izone = 1;
           irpm = 0;
        }

     }

  }

  protected void set_colors() {
    
    if (Robot.intake.isIntakeOn() && (getGamedata() != ""))  { // Intake On, Have Game Data
      if (iloops <= 50){
        Robot.led.setWhite();
        iloops = iloops +1;    
      } else {
        setColorWheel();
        iloops = iloops +1;
    
        if (iloops == 100){
          iloops = 0;
        }
      }
    } else if (Robot.intake.isIntakeOn() && (getGamedata() == "")) { // Intake On, no Game Data  
          Robot.led.setWhite(); 
    } else if (Robot.limelight.is_Target() && (getGamedata() != "")){ // Have Target, Have Game Data
      if (iloops <= 50){
        Robot.led.setViolet();
        iloops = iloops +1;    
      } else {
        setColorWheel();
        iloops = iloops +1;
        if (iloops == 100){
          iloops = 0;
        }
      }
          
    } else if (Robot.limelight.is_Target() && (getGamedata() == "")){ // Have Target, No Game Data
      Robot.led.setViolet();
    } else { // Lights Off
      if (Robot.drive.IsLow()) {
        Robot.led.setLavaWave();
      } else {   
         Robot.led.setBlack();
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
