/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.subsystems.Subsystem_Compressor;
import frc.robot.subsystems.Subsystem_Drive;
import frc.robot.subsystems.Subsystem_Elevator;
import frc.robot.subsystems.Subsystem_Hopper;
import frc.robot.subsystems.Subsystem_Intake;
import frc.robot.subsystems.Subsystem_LED;
import frc.robot.subsystems.Subsystem_Limelight;
import frc.robot.subsystems.Subsystem_Shooter;
import frc.robot.subsystems.Subsystem_Winch;

public class Robot extends TimedRobot {

  //declares subsystem variables
  public static Subsystem_Drive drive = new Subsystem_Drive();
  public static Subsystem_Shooter shooter = new Subsystem_Shooter();
  public static Subsystem_Compressor compressor = new Subsystem_Compressor();
  public static Subsystem_Elevator elevator = new Subsystem_Elevator();
  public static Subsystem_Hopper hopper = new Subsystem_Hopper();
  public static Subsystem_Intake intake = new Subsystem_Intake();  
  public static Subsystem_Winch winch = new Subsystem_Winch();

  public static Subsystem_Limelight limelight = new Subsystem_Limelight();

  public static Subsystem_LED led = new Subsystem_LED();


  private double Kp = -0.03f;
  private double Ki = 0.012f; // 0.006
  private double Kf = 0.05f;  //feedforward - minimum command signal
  
  private double left_command;
  private double right_command;
  
  private double x, thor;
  private int izone, irpm;
  private boolean btarget;
  private double zone1threshold = 50; //  pixel width
  public double distance;
  private int iloops = 0;
  
  private int isuccess = 0; 

  private String cstate = "HUNT";

  private int caseMove = 0;



  

  //public static Subsystem_Limelight limelight = new Subsystem_Limelight();
  public static UsbCamera camera;
  public static OI oi;

  SendableChooser<Integer> autoChooser = new SendableChooser<>();

  //Run when the robot is first started
  @Override
  public void robotInit() {

    //generateTrajectories();

    //resets the encoder on initialize
    //Robot.tower.resetEncoder();
    oi = new OI();

    //starts camera stream if camera is available
    try {

      camera = CameraServer.getInstance().startAutomaticCapture();
    }
    catch(Exception ex) {

      System.out.println("ERROR: setting camera: " + ex.getMessage()) ;
    } 


    autoChooser.setDefaultOption("Auto1", 1);
    autoChooser.addOption("Auto2", 2);
    // etc.
    SmartDashboard.putData("Autonomous routine", autoChooser);




  }

  //This function is a loop that is always running
  @Override
  public void robotPeriodic() {
  }

  //Called each time the robot is disabled
  @Override
  public void disabledInit() {
  }

  //Called periodically when robot is disabled
  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  //Called when autonomous is initalised
  @Override
  public void autonomousInit() {

    //puts robot into low gear once auto/sandstorm starts
  }

  //This function is called periodically during autonomous
  @Override
  public void autonomousPeriodic() {

    switch (cstate) {  
      case "HUNT" : 
        Robot.drive.LowGear();

       distance = Robot.limelight.getDistance();
       SmartDashboard.putNumber("thedistance", distance);


       thor = Robot.limelight.get_Thor();
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
            SmartDashboard.putNumber("thedistance", distance );

            if (distance > 0){
              if (isuccess >= 5) {
              cstate = "SHOOT";
              
              }
              SmartDashboard.putNumber("isuccess", isuccess);
            }
          }
            
       } // if btarget

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

         else if ((distance > 300) && (distance <= 400)){//259, 450
           irpm = 5000;
           Robot.shooter.hoodUp();
           Robot.shooter.setShooterPIDTrench();
         }

         else if ((distance > 4000) && (distance <= 5000)){//259, 450
          irpm = 4250;
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

            caseMove = caseMove + 1;

            if (caseMove >= 100){
              cstate = "MOVE";
              caseMove = 0;
            }
         } 

         break;
      case "MOVE" : 
      Robot.drive.curvaturedrive(-0.3, 0); // backwards at slow speed
/*
      if (Robot.drive.getEncLeftSide() > -10){
        Robot.drive.setLeftSide(0.3);

      }

      if (Robot.drive.getEncRightSide() < 10){
        Robot.drive.setRightSide(-0.3);

      } */
    }  


    //Autonomous Commands Using ShuffleBoard

    int autoMode = autoChooser.getSelected();
    // Run the appropriate command

    
    if (autoMode == 1){

    }

    else if (autoMode == 2){


    }

    else if (autoMode == 3){


    }

    else{

    }

  
  }

  //Called when teleop is initialised
  @Override
  public void teleopInit() {
    Robot.shooter.setShooterSpeed(0);
    Robot.elevator.elevatorStop();
    Robot.hopper.hopperStop();
    Robot.intake.stopIntake();
    Robot.drive.setRightSide(0);
    Robot.drive.setLeftSide(0);
  }

  //This function is called periodically during operator control
  @Override
  public void teleopPeriodic() {
    //Robot.shooter.ShooterSpeed(500);

    Scheduler.getInstance().run();
    //winch.UpdateLimitSwitch();
  }

  //This function is called periodically during test mode
  @Override
  public void testPeriodic() {
  }

  /*
  public static void generateTrajectories() {

    paths.put("Gather", GatherFromTrench.generate());
    SmartDashboard.putBoolean("Paths Generated", true);
  }
  */

  
}
