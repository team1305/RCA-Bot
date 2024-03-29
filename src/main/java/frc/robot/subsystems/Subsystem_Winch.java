/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
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

  
  private final WPI_TalonFX mtWinch1 = RobotMap.mtWinch1;
  private final WPI_TalonFX mtWinch2 = RobotMap.mtWinch2;
  private final Solenoid slHook = RobotMap.slndHook;

  private boolean isHookUp;



  
  public Subsystem_Winch() {

    mtWinch1.configFactoryDefault();
    mtWinch2.configFactoryDefault();

    mtWinch1.setNeutralMode(NeutralMode.Brake);
    mtWinch2.setNeutralMode(NeutralMode.Brake);

    mtWinch1.configOpenloopRamp(0.4);
    mtWinch2.configOpenloopRamp(0.4);

    mtWinch1.setInverted(true);
    mtWinch1.setSensorPhase(true);

    mtWinch2.setInverted(false);
    mtWinch2.setSensorPhase(true);

    mtWinch1.configStatorCurrentLimit(RobotMap.currentLimitConfig30, 40);
    mtWinch2.configStatorCurrentLimit(RobotMap.currentLimitConfig30, 40);

    mtWinch2.follow(mtWinch1);

    isHookUp = false;



  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }


  public void WinchSpeed(double speed) {
    mtWinch1.set(speed);
    mtWinch2.set(speed);
  }

  

  public void stopWinch() {
    mtWinch1.set(0);
    mtWinch2.set(0);
  }

  public void hookUp(){
    isHookUp = true;
    slHook.set(true);
  }

  public void hookDown(){
    isHookUp=false;
    slHook.set(false);
  }

  public void hookToggle(){
    if (isHookUp){
      hookDown();
    }
    else{
      hookUp();
    }
  }

  

}


