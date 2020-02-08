/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Subsystem_Hopper extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static CANSparkMax mtHopper = RobotMap.mtHopper;

  public Subsystem_Hopper() {

    
    mtHopper.restoreFactoryDefaults();
    mtHopper.setIdleMode(IdleMode.kCoast);
    mtHopper.setOpenLoopRampRate(0.4);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void hopperIn(double speed) {
    mtHopper.set(-speed);    
  }

  public void hopperOut(double speed) {

    mtHopper.set(speed);
  }

  public void hopperStop() {
    mtHopper.set(0);
  }
}
