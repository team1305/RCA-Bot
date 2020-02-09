/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Collection;
import java.util.HashSet;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;


/**
 * Add your docs here.
 */
public class Subsystem_Orchestra extends Subsystem {

  //grabs drive motor information from RobotMap
  private final WPI_TalonFX mtLeft1 = RobotMap.mtDriveLeft1;
  private final WPI_TalonFX mtLeft2 = RobotMap.mtDriveLeft2;
  private final WPI_TalonFX mtRight1 = RobotMap.mtDriveRight1;
  private final WPI_TalonFX mtRight2 = RobotMap.mtDriveRight2;

  private final Collection< TalonFX > instruments = new HashSet< TalonFX >();
  private final Orchestra starwars;

  private String filePath;

  
  public Subsystem_Orchestra() {

    instruments.add(mtLeft1);
    instruments.add(mtLeft2);
    instruments.add(mtRight1);
    instruments.add(mtRight2);

    filePath = "/home/lvuser/deploy/starwars.chirp";

    starwars = new Orchestra(instruments, filePath);
    
  }

  
  @Override
  public void initDefaultCommand() {
    //unless interupted the default command will allow driver to drive with joystick
    //setDefaultCommand(new Command_Drive_With_Joystick());
  }

  public void playSong(){
    starwars.play();
  }

  public void pauseSong(){
    starwars.pause();
  }

  public void stopSong(){
    starwars.stop();
  }

  public boolean isPlaying(){
    return starwars.isPlaying();
  }

}

