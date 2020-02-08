/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Subsystem_Winch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  ////public static CANSparkMax mtWicnh1 = RobotMap.mtWicnh1;
  ////public static CANSparkMax mtWicnh2 = RobotMap.mtWicnh2;

  
  private final WPI_TalonFX mtWicnh1 = RobotMap.mtWicnh1;
  private final WPI_TalonFX mtWicnh2 = RobotMap.mtWicnh2;



  
  public Subsystem_Winch() {

    mtWicnh1.setNeutralMode(NeutralMode.Brake);
    mtWicnh2.setNeutralMode(NeutralMode.Brake);

    mtWicnh1.configOpenloopRamp(0.4);
    mtWicnh2.configOpenloopRamp(0.4);

    mtWicnh1.configFactoryDefault();
    mtWicnh2.configFactoryDefault();

    mtWicnh1.setInverted(false);
    mtWicnh1.setSensorPhase(true);

    mtWicnh2.setInverted(true);
    mtWicnh2.setSensorPhase(true);


  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void WinchSpeed(double speed) {
    mtWicnh1.set(-speed);
    mtWicnh2.set(-speed);
  }

  public void stopWinch() {
    mtWicnh1.set(0);
    mtWicnh2.set(0);
  }

}


