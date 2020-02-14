/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Subsystem_Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static WPI_TalonFX mtIntake = RobotMap.mtIntake;
  public static Solenoid slndIntake = RobotMap.slndIntake;


  private boolean intakeOn = false;

  public Subsystem_Intake() {
    
    mtIntake.configFactoryDefault();
    mtIntake.setNeutralMode(NeutralMode.Coast);
    mtIntake.configOpenloopRamp(0.4);
    mtIntake.configStatorCurrentLimit(RobotMap.currentLimitConfig, 40);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void enableIntake(double speed) {
    mtIntake.set(ControlMode.PercentOutput, -speed);

  }

  public void reverseIntake(double speed) {
    mtIntake.set(ControlMode.PercentOutput, speed);
  }

  public void stopIntake() {
    mtIntake.set(ControlMode.PercentOutput, 0);
  }


  public void extendIntake() {
    slndIntake.set(true);
    intakeOn = true;
  }

  public void retractIntake() {
    slndIntake.set(false);
    intakeOn = false;

  }


  public boolean isIntakeOn(){
    return intakeOn;
  }
  
}


