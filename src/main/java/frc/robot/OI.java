/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.InternalButton;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.Command_Compressor_OFF;
import frc.robot.commands.Command_Compressor_ON;
import frc.robot.commands.Command_Intake_Ground;
import frc.robot.commands.Command_Intake_Station;
import frc.robot.commands.Command_Shift_Gear;
import frc.robot.commands.Command_Shoot;
import frc.robot.commands.Command_Supply_Balls;
import frc.robot.commands.Elevator_Down;
import frc.robot.commands.Elevator_Up;
import frc.robot.commands.Hopper_In;
import frc.robot.commands.Hopper_Out;
import frc.robot.commands.Intake_Out;
import frc.robot.commands.Intake_Spin;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  //declares joysticks
  public Joystick joyxbox1 = new Joystick(0);  // Primary Driver Stick
  public Joystick joyxbox2 = new Joystick(1);  // Operator Control Stick
  
  //declares button mapping of the joyxbox1 controller
	Button btn1_A = new JoystickButton(joyxbox1, 1);  //xbox "A" Button 1
	Button btn1_B = new JoystickButton(joyxbox1, 2); //xbox "B" Button 2
	Button btn1_X = new JoystickButton(joyxbox1, 3); //xbox "X" Button 3
	Button btn1_Y = new JoystickButton(joyxbox1, 4); //xbox "Y" Button 4
	Button btn1_LB = new JoystickButton(joyxbox1, 5); //xbox "LB" Button 5
	Button btn1_RB = new JoystickButton(joyxbox1, 6);  //xbox "RB" Button 6
	Button btn1_Back = new JoystickButton(joyxbox1, 7);  //xbox "Back" Button 7
	Button btn1_Start = new JoystickButton(joyxbox1, 8);  //xbox "Start" Button 8
	Button btn1_LS = new JoystickButton(joyxbox1, 9);  //xbox "Left Stick Click" Button 9
  Button btn1_RS = new JoystickButton(joyxbox1, 10);  //xbox "Right Stick Click" Button 10

  //declares button mapping of the joyxbox2 controller
	Button btn2_A = new JoystickButton(joyxbox2, 1);  //xbox "A" Button 1
	Button btn2_B = new JoystickButton(joyxbox2, 2); //xbox "B" Button 2
	Button btn2_X = new JoystickButton(joyxbox2, 3); //xbox "X" Button 3
	Button btn2_Y = new JoystickButton(joyxbox2, 4); //xbox "Y" Button 4
	Button btn2_LB = new JoystickButton(joyxbox2, 5); //xbox "LB" Button 5
	Button btn2_RB = new JoystickButton(joyxbox2, 6);  //xbox "RB" Button 6
	Button btn2_Back = new JoystickButton(joyxbox2, 7);  //xbox "Back" Button 7
	Button btn2_Start = new JoystickButton(joyxbox2, 8);  //xbox "Start" Button 8
	Button btn2_LS = new JoystickButton(joyxbox2, 9);  //xbox "Left Stick Click" Button 9
  Button btn2_RS = new JoystickButton(joyxbox2, 10);  //xbox "Right Stick Click" Button 10

  // InternalButton btn_Spear = new InternalButton();

  // public void SetSpearButton(Boolean s) {
  //     btn_Spear.setPressed(s);
  // }

  public OI() {
 
    // Primary Driver Stick
    //btn1_A.whileHeld(new Elevator_Down());//xbox "A" Button 1

    //btn1_B.whileHeld(new Elevator_Up());//xbox "B" Button 2

    //btn1_X.whileHeld(new Hopper_In());//xbox "X" Button 3

    //btn1_Y.whileHeld(new Hopper_Out());//xbox "Y" Button 4

    //btn1_RB.whileHeld(new Hopper_Out());//xbox "RB" Button 6

    btn1_Back.whenPressed(new Command_Compressor_OFF());//xbox "Back" Button 7
    btn1_Start.whenPressed(new Command_Compressor_ON());//xbox "Start" Button 8
    btn1_LS.whenPressed(new Command_Shift_Gear());//xbox "Left Stick Click" Button 9

    //btn1_RS.whileHeld(new Intake_Out());//xbox "Right Stick Click" Button 10
    //xbox "X Axis" Left Stick - 
    ////xbox "Y Axis" Left Stick - Drive Forward and Reverse
    ////xbox "X Axis 5" Right Stick - 
    ////xbox "Y Axis 4" Right Stick - Drive Left and Right

    ////xbox "Axis 3" Left Trigger - 
    ////xbox "Axis 3" Right Trigger - 
    ////xbox "Up" Direction Pad -
    ////xbox "Down" Direction Pad -
    ////xbox "Left" Direction Pad -
    ////xbox "Right" Direction Pad -

    // Secondary Driver Stick
    
    /*
    btn2_A.whileHeld(new Elevator_Down());//xbox "A" Button 1
    btn2_B.whileHeld(new Elevator_Up());//xbox "B" Button 2
    btn2_X.whileHeld(new Hopper_In());//xbox "X" Button 3
    btn2_Y.whileHeld(new Hopper_Out());//xbox "Y" Button 4
    btn2_LB.whileHeld(new Intake_Spin());//xbox "LB" Button 5
    btn2_RB.whileHeld(new Intake_Out());//xbox "RB" Button 6
    */



    
    btn2_A.whileHeld(new Command_Intake_Ground());
    btn2_B.whileHeld(new Command_Intake_Station());
    btn2_RB.whileHeld(new Command_Shoot());
    btn2_LB.whileHeld(new Command_Supply_Balls());
    



  

    //btn2_Back.whileHeld(new Command_Unwinch());//xbox "Back" Button 7
    //btn2_Start.whileHeld(new Command_Winch());//xbox "Start" Button 8
    //btn2_LS.whenPressed(new Command_Toggle_Kicker());//xbox "Left Stick Click" Button 9
    //btn2_RS.whileHeld(new Command_Toggle_Kicker());//xbox "Right Stick Click" Button 10
    ////xbox "X Axis" Left Stick - 
    ////xbox "Y Axis" Left Stick -
    ////xbox "X Axis 5" Right Stick - 
    ////xbox "Y Axis 4" Right Stick -
    ////xbox "Axis 3" Left Trigger - Kick Hatch
    ////xbox "Axis 3" Right Trigger - tower mid level
    ////xbox "Up" Direction Pad - Tower Forward
    ////xbox "Down" Direction Pad - Tower Backward
    ////xbox "Left" Direction Pad - Tower Leftside
    ////xbox "Right" Direction Pad - Tower Rightside

    // if (btn_Spear.get() == false) {
    //   // new Command_Spear();
    // }
  }
  
  //returns joyxbox1 whenever getJoystickDriver is called
  public Joystick getJoystickDriver() {
    return joyxbox1;
  }

  //returns joyxbox2 whenever getJoystickOperator is called
  public Joystick getJoystickOperator() {
    return joyxbox2;
  }
}
