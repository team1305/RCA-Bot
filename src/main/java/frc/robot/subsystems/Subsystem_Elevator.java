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

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Subsystem_Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static WPI_TalonFX mtElevator = RobotMap.mtElevator;

  public Subsystem_Elevator() {

    
    mtElevator.configFactoryDefault();
    mtElevator.setNeutralMode(NeutralMode.Coast);
    mtElevator.configOpenloopRamp(0.4);
    mtElevator.configStatorCurrentLimit(RobotMap.currentLimitConfig30, 30);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }


  public void elevatorUp(double speed) {
    mtElevator.set(ControlMode.PercentOutput, speed);
  }


  public void elevatorDown(double speed) {
    mtElevator.set(ControlMode.PercentOutput, -speed);
  }

  public void elevatorStop() {
    mtElevator.set(ControlMode.PercentOutput, 0);
  }
}
