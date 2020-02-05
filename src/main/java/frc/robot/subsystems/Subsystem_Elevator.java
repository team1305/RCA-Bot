/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Subsystem_Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static CANSparkMax mtElevator = RobotMap.mtElevator;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }


  public void elevatorUp() {
    mtElevator.set(-0.5);
  }


  public void elevatorDown() {
    mtElevator.set(0.5);
  }

  public void elevatorStop() {
    mtElevator.set(0);
  }
}
