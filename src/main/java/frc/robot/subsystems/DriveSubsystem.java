/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveSubsystem.
   */

  //Drive Motors
  public static WPI_TalonFX leftTalon1 = new WPI_TalonFX(Constants.leftMotorPort1);
  public static WPI_TalonFX leftTalon2 = new WPI_TalonFX(Constants.leftMotorPort2);

  public static WPI_TalonFX rightTalon1 = new WPI_TalonFX(Constants.rightMotorPort1);
  public static WPI_TalonFX rightTalon2 = new WPI_TalonFX(Constants.rightMotorPort2);

  SpeedControllerGroup sgcLeft = new SpeedControllerGroup(leftTalon1, leftTalon2);
  SpeedControllerGroup sgcRight = new SpeedControllerGroup(rightTalon1, rightTalon2);

  DifferentialDrive dr = new DifferentialDrive(sgcLeft, sgcRight);
  
  public DriveSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveWJoystick(double leftStickY, double rightStickX) {  
    dr.curvatureDrive(leftStickY, rightStickX, true);
  }

  public void halt() {
    dr.tankDrive(0, 0);
  }



}
