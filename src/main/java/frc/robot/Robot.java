/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.auto_commands.AutoDrivePID;
import frc.robot.commands.AutoCommands;
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

  Command autonomousCommand;
  SendableChooser<CommandGroup> autonomousModes;


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

  //private AutoDrivePID autodrive = new AutoDrivePID(100000, Robot.drive);

  

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
    /*try {

      camera = CameraServer.getInstance().startAutomaticCapture();
    }
    catch(Exception ex) {

      System.out.println("ERROR: setting camera: " + ex.getMessage()) ;
    } */


    
    autonomousModes = new SendableChooser<CommandGroup>();
    autonomousModes.setDefaultOption("6 ball auto, straight lined up, our trench", new AutoCommands(1));
    autonomousModes.addOption("8 ball auto, straight lined up, our trench", new AutoCommands(2));
    autonomousModes.addOption("8 ball auto, opposite side steal, then generator", new AutoCommands(3));

    SmartDashboard.putData("AUTO Modes", autonomousModes);
    Robot.drive.HighGear();
    Robot.drive.resetEncoder();
    Robot.drive.gyroReset();



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
    Robot.drive.gyroReset();
    Robot.drive.resetEncoder();


    	autonomousCommand = (Command) autonomousModes.getSelected();
    	if (autonomousCommand != null) autonomousCommand.start();
   //   Robot.drive.gyroReset();

    //puts robot into low gear once auto/sandstorm starts
  }

  //This function is called periodically during autonomous
  @Override
  public void autonomousPeriodic() {

    Scheduler.getInstance().run();

  
  }

  //Called when teleop is initialised
  @Override
  public void teleopInit() {

    if (autonomousCommand != null) autonomousCommand.cancel();

    Robot.shooter.setShooterSpeed(0);
    Robot.elevator.elevatorStop();
    Robot.hopper.hopperStop();
    Robot.intake.stopIntake();
    Robot.drive.setRightSide(0);
    Robot.drive.setLeftSide(0);
    Robot.limelight.limelightOff();
  }

  //This function is called periodically during operator control
  @Override
  public void teleopPeriodic() {
    //Robot.shooter.ShooterSpeed(500);

    Scheduler.getInstance().run();
    //winch.UpdateLimitSwitch();
    //Robot.led.setOcean();
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
