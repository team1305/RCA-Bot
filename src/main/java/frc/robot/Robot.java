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
import frc.robot.trajectories.GatherFromTrench;

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



  public static HashMap<String, Trajectory> paths = new HashMap<>();

  

  //public static Subsystem_Limelight limelight = new Subsystem_Limelight();
  public static UsbCamera camera;
  public static OI oi;

  //Run when the robot is first started
  @Override
  public void robotInit() {

    generateTrajectories();

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

    //Autonomous Commands Using ShuffleBoard

    
    if (SmartDashboard.getBoolean("Center", true)){


    }

    else if (SmartDashboard.getBoolean("Right", true)){


    }

    else if (SmartDashboard.getBoolean("Left", true)){


    }

    else{

    }

  
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


  public static void generateTrajectories() {

    paths.put("Gather", GatherFromTrench.generate());
    SmartDashboard.putBoolean("Paths Generated", true);
  }

  public Subsystem_Drive getDrive() {
    return drive;
  }
}
