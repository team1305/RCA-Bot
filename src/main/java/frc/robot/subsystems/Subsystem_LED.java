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
public class Subsystem_LED extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final CANSparkMax mtled = RobotMap.mtLed;
  

  private final double C_blue = 0.87;
  private final double C_green = 0.77;
  private final double C_red = 0.61;
  private final double C_yellow = 0.69;
  private final double C_aqua = 0.81;
  private final double C_white = 0.93;
  private final double C_violet = 0.91;
  private final double C_black = 0.99;



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setBlue(){
    mtled.set(C_blue);
  }

  public void setGreen(){
    mtled.set(C_green);
  }

  public void setRed(){
    mtled.set(C_red);
  }

  public void setYellow(){
    mtled.set(C_yellow);
  }

  public void setAqua(){
    mtled.set(C_aqua);
  }

  public void setWhite(){
    mtled.set(C_white);
  }

  public void setViolet(){
    mtled.set(C_violet);
  }

  public void setBlack(){
    mtled.set(C_black);
  }




  
}
