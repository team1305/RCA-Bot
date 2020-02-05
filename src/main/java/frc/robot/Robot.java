/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Subsystem_Compressor_Power;
import frc.robot.subsystems.Subsystem_Drive;
import frc.robot.subsystems.Subsystem_Elevator;
import frc.robot.subsystems.Subsystem_Hopper;
import frc.robot.subsystems.Subsystem_Intake;
import frc.robot.subsystems.Subsystem_Shooter;


public class Robot extends TimedRobot {

  //declares subsystem variables
  public static Subsystem_Drive drive = new Subsystem_Drive();
  public static Subsystem_Shooter shooter = new Subsystem_Shooter();
  public static Subsystem_Compressor_Power compressor = new Subsystem_Compressor_Power();
  public static Subsystem_Elevator elevator = new Subsystem_Elevator();
  public static Subsystem_Hopper hopper = new Subsystem_Hopper();
  public static Subsystem_Intake intake = new Subsystem_Intake();
  
  //public static Subsystem_Tower_Rotation tower = new Subsystem_Tower_Rotation();
  //public static Subsystem_Intake intake = new Subsystem_Intake();
  //public static Subsystem_Winch winch = new Subsystem_Winch();
  //public static Subsystem_Tower_Lift towerLift = new Subsystem_Tower_Lift();
  //public static Subsystem_RGBLED_CAN rgbLedController = new Subsystem_RGBLED_CAN();
  //public static Subsystem_Limelight limelight = new Subsystem_Limelight();
  public static UsbCamera camera;
  public static OI oi;

  //Run when the robot is first started
  @Override
  public void robotInit() {

    //resets the encoder on initialize
    //Robot.tower.resetEncoder();
    oi = new OI();

    //Displays information of robot status
    //SmartDashboard.putBoolean("Tower Level 1", towerLift.bTowerLowerIsUp); 
    //SmartDashboard.putBoolean("Tower Level 2", towerLift.bTowerUpperIsUp);
    //SmartDashboard.putBoolean("Intake", intake.bWristIsDown);
    //SmartDashboard.putBoolean("J Hook", intake.bHatchGrabbed);
    //SmartDashboard.putBoolean("Gear", drive.bIsLow);
    //Robot.rgbLedController.RGBledCAN();

    //starts camera stream if camera is available
    try {
      camera = CameraServer.getInstance().startAutomaticCapture();
    }
    catch(Exception ex) {
      System.out.println("ERROR: setting camera: " + ex.getMessage()) ;
    } 
    
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
    Robot.drive.LowGear();
  }

  //This function is called periodically during autonomous
  @Override
  public void autonomousPeriodic() {

    //in auto/sandstorm calls teleop
    teleopPeriodic();
  }

  //Called when teleop is initialised
  @Override
  public void teleopInit() {
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
}
