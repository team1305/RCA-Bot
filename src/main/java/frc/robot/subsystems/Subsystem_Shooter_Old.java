/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Command_ai_loop;


/**
 * Add your docs here.
 */

/* 
public class Subsystem_Shooter_Old extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final WPI_TalonFX mtShootLeft = RobotMap.mtShootLeft1;
  private final WPI_TalonFX mtShootRight = RobotMap.mtShootRight1;

  public final int timeOutMs = 30;
  public final int loopIDX = 0;//Leave as zero
  public final double f = 0; //1023 is actually 100% 'f' value for these motors.
  public final double p = 0; //.25;
  public final double i = 0; //.001;
  public final double d = 0; //20;  



  public Subsystem_Shooter_Old() {
    mtShootLeft.configFactoryDefault();
    mtShootRight.configFactoryDefault();

    // Set Motors to Neutral mode    
    mtShootLeft.setNeutralMode(NeutralMode.Coast);
    mtShootRight.setNeutralMode(NeutralMode.Coast);  

    // Invert Right Motor
    mtShootRight.setInverted(true);
    // Set Right Motor to follow Left motor
    //mtShootRight.set(TalonFXControlMode.Follower, 5);
    
    // Set Ramp Rates
    //mtShootLeft.setOpenLoopRampRate(0.4);
    //mtShootRight.setOpenLoopRampRate(0.4);

    mtShootLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, loopIDX, timeOutMs); //INTEGRATEDSENSOR maybe a solution||||||||||||||||||||||||||
    mtShootRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, loopIDX, timeOutMs);


    mtShootLeft.config_kF(loopIDX, f, timeOutMs);
    mtShootLeft.config_kP(loopIDX, p, timeOutMs);
    mtShootLeft.config_kI(loopIDX, i, timeOutMs);
    mtShootLeft.config_kD(loopIDX, d, timeOutMs);

    mtShootRight.config_kF(loopIDX, f, timeOutMs);
    mtShootRight.config_kP(loopIDX, p, timeOutMs);
    mtShootRight.config_kI(loopIDX, i, timeOutMs);
    mtShootRight.config_kD(loopIDX, d, timeOutMs);


  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Command_ai_loop());

  }

  public void ShooterSpeed(double ispeed) {

    double motorRPMR = mtShootRight.getSelectedSensorVelocity();
    double motorRPML = mtShootLeft.getSelectedSensorVelocity();

    mtShootLeft.set(ControlMode.PercentOutput, ispeed);     
    mtShootRight.set(ControlMode.PercentOutput, ispeed);

    SmartDashboard.putNumber("ShooterSpeedR", motorRPMR);
    SmartDashboard.putNumber("ShooterSpeedL", motorRPML);

    
   }

   public void ShooterStop() {
    
    mtShootLeft.set(ControlMode.Velocity, 0);
    mtShootRight.set(ControlMode.Velocity, 0);


    int timeOutMs = 30;
    int loopIDX = 0;
    double f = 0;
    double p = 0;
    double i = 0;
    double d = 0;  

    mtShootLeft.config_kF(loopIDX, f, timeOutMs);
    mtShootLeft.config_kP(loopIDX, p, timeOutMs);
    mtShootLeft.config_kI(loopIDX, i, timeOutMs);
    mtShootLeft.config_kD(loopIDX, d, timeOutMs);

    mtShootRight.config_kF(loopIDX, f, timeOutMs);
    mtShootRight.config_kP(loopIDX, p, timeOutMs);
    mtShootRight.config_kI(loopIDX, i, timeOutMs);
    mtShootRight.config_kD(loopIDX, d, timeOutMs);
  }
}

*/






