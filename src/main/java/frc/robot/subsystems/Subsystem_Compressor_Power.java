/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Subsystem_Compressor_Power extends Subsystem {
 
  //grabs device ID from robotmap
  private final Compressor compressor = RobotMap.cmpRobotCompressor;

  
  @Override
  public void initDefaultCommand() {
       
  }

  //turns the compressor on
  public void CompressorON() {

    compressor.start();
  }

  //checks to see if the compressor is enabled
  public boolean isenabled() {
    return compressor.enabled();
  }

  //turns the compressor off
  public void CompressorOFF() {

    compressor.stop();
  }
}
