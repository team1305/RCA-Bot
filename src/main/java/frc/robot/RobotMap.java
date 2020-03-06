/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//import com.ctre.phoenix.CANifier;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.revrobotics.CANEncoder;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;

public class RobotMap {

  // declares sparkmax motor controllers used and digital ID of motor
  public static WPI_TalonFX mtDriveLeft1 = new WPI_TalonFX(1);
  public static WPI_TalonFX mtDriveLeft2 = new WPI_TalonFX(2);
  public static WPI_TalonFX mtDriveRight1 = new WPI_TalonFX(3);
  public static WPI_TalonFX mtDriveRight2 = new WPI_TalonFX(4);

  public static WPI_TalonFX mtShootLeft1 = new WPI_TalonFX(11);
  public static WPI_TalonFX mtShootRight1 = new WPI_TalonFX(12);
  public static WPI_TalonFX mtShootRight2 = new WPI_TalonFX(13);


  public static WPI_TalonFX mtIntake = new WPI_TalonFX(7);
  public static WPI_TalonFX mtElevator = new WPI_TalonFX(6);
  public static WPI_TalonFX mtHopper = new WPI_TalonFX(5);

  public static WPI_TalonFX mtWinch1 = new WPI_TalonFX(9);
  public static WPI_TalonFX mtWinch2 = new WPI_TalonFX(10);


  public static Spark mtLed = new Spark(9);


   public final static StatorCurrentLimitConfiguration currentLimitConfig = new StatorCurrentLimitConfiguration(true, 40, 39.95, 1);
   public final static StatorCurrentLimitConfiguration currentLimitConfig30 = new StatorCurrentLimitConfiguration(true, 30, 29.95, 1);
    
  //Shooter PID values
     public static final double kP_SHOOTER_INFRONT_OF_LINE = 0.095;  //0.085
     public static final double kI_SHOOTER_INFRONT_OF_LINE = 0;
     public static final double kD_SHOOTER_INFRONT_OF_LINE = 0;
     public static final double kF_SHOOTER_INFRONT_OF_LINE = 0.0512;

     public static final double kP_SHOOTER_INITIATION_LINE = 0.09;
     public static final double kI_SHOOTER_INITIATION_LINE = 0;
     public static final double kD_SHOOTER_INITIATION_LINE = 0;
     public static final double kF_SHOOTER_INITIATION_LINE = 0.0512;

     public static final double kP_SHOOTER_TRENCH = 0.1;
     public static final double kI_SHOOTER_TRENCH = 0;
     public static final double kD_SHOOTER_TRENCH = 0;
     public static final double kF_SHOOTER_TRENCH = 0.0512;

     public static final double kP_SHOOTER_TRENCH_BACK = 0.1;
     public static final double kI_SHOOTER_TRENCH_BACK = 0;
     public static final double kD_SHOOTER_TRENCH_BACK = 0;
     public static final double kF_SHOOTER_TRENCH_BACK = 0.0512;



     public static final int kIZone_SHOOTER = 200;
     public static final double SHOOTER_OUTPUT_TO_ENCODER_RATIO = 0.77; //Previous 3.0 Because 3 revolutions of the encoder was one revolution of the wheels, 24.0/36.0
     public static final double TICKS_PER_ROTATION = 2048.0;
     public static final int kLongCANTimeOutMs = 100;
     public static final double kFlywheelTicksPerRevolution = 0;

  //public static CANSparkMax mtWinch1 = new CANSparkMax(5, MotorType.kBrushless);
  //public static CANSparkMax mtWinch2 = new CANSparkMax(6, MotorType.kBrushless);
  //public static CANSparkMax mtIntake = new CANSparkMax(7, MotorType.kBrushless);
  //public static CANSparkMax mtTowerLift = new CANSparkMax(8, MotorType.kBrushless);

  //declares encoders for motor controllers and motors connected to encoder
  /*
  public static CANEncoder enDriveLeft1 = new CANEncoder(mtDriveLeft1);
  public static CANEncoder enDriveLeft2 = new CANEncoder(mtDriveLeft2);
  public static CANEncoder enDriveRight1 = new CANEncoder(mtDriveRight1);
  public static CANEncoder enDriveRight2 = new CANEncoder(mtDriveRight2);
  public static CANEncoder enWinch1 = new CANEncoder(mtWinch1);
  public static CANEncoder enTowerLift = new CANEncoder(mtTowerLift);
  */

  // DIO 
  public static final DigitalInput ballInSensor = new DigitalInput(1);
  public static final DigitalInput ballMaxSensor = new DigitalInput(2);

  //declares CANifier for LED control
  //public static CANifier RGBLEDController = new CANifier(20);

  //declares compressor port
  public static Compressor cmpRobotCompressor = new Compressor(0);

  //declares digital id for tower motor 
  //public static WPI_TalonSRX mtTowerRotate = new WPI_TalonSRX(0);

  //declares port number of solenoid
  public static Solenoid slndGearShifter = new Solenoid(0);

  public static Solenoid slndHook =  new Solenoid(6);

  // 2 is empty

  // 3 is empty

  public static Solenoid slndIntake =  new Solenoid(4);

  public static Solenoid slndHood =  new Solenoid(5);

  // 6 is empty

  //7 is vent



  //runs on initialize
  public static void init() {    

  }



}



