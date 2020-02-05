/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


//import com.revrobotics.CANEncoder;
//import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.Command_Drive_With_Joystick;



/**
 * Add your docs here.
 */
public class Subsystem_Drive extends Subsystem {

  //variable for deadband
  public double dDeadband = 0.1;
  public double dSquareFactor = 1.0;
  public double dThrottleFactor = 0.5;

  //boolean for checking gear shift state
  public boolean bIsLow = false;

  //grabs drive motor information from RobotMap
  private final WPI_TalonFX mtLeft1 = RobotMap.mtDriveLeft1;
  private final WPI_TalonFX mtLeft2 = RobotMap.mtDriveLeft2;
  private final WPI_TalonFX mtRight1 = RobotMap.mtDriveRight1;
  private final WPI_TalonFX mtRight2 = RobotMap.mtDriveRight2;

  //grabs drive encoder information from RobotMap
  //private final CANEncoder enLeft1 = RobotMap.enDriveLeft1;
  //private final CANEncoder enLeft2 = RobotMap.enDriveLeft2;
  //private final CANEncoder enRight1 = RobotMap.enDriveRight1;
  //private final CANEncoder enRight2 = RobotMap.enDriveRight2;

  //grabs solenoid for gear shifting
  private final Solenoid slndShift = RobotMap.slndGearShifter;


  //creates motor controller groups for left and right motors
  SpeedControllerGroup scgLeft = new SpeedControllerGroup(mtLeft1, mtLeft2);
  SpeedControllerGroup scgRight = new SpeedControllerGroup(mtRight1, mtRight2);




  // Internal subsystem parts, declares left and right motor groups used by differential drive
  DifferentialDrive drRobotDrive = new DifferentialDrive(scgLeft, scgRight);

  // Subsystem State Value
  private Double enLeftStart;
  private Double enRightStart;
  
  //sets ramprate of drive motors -- now does things!
  public Subsystem_Drive() {
    //Trying to get brake mode working
    mtLeft1.setNeutralMode(NeutralMode.Brake);
    mtLeft2.setNeutralMode(NeutralMode.Brake);


    mtLeft1.configOpenloopRamp(0.4);
    mtLeft2.configOpenloopRamp(0.4);
    mtRight1.configOpenloopRamp(0.4);
    mtRight2.configOpenloopRamp(0.4);

    // SmartDashboard.putNumber("dSquareFactor", dSquareFactor);
    // SmartDashboard.putNumber("dThrottleFactor", dThrottleFactor);
  }

  @Override
  public void initDefaultCommand() {
    //unless interupted the default command will allow driver to drive with joystick
    setDefaultCommand(new Command_Drive_With_Joystick());
  }

  //creates a deadband for the joystick so that the robot does not spin
  // when nobody is touching the controls
  private double JoystickDeadBand(final double input) {
    if(Math.abs(input) < dDeadband) return 0;
    else if(input > 0) return Math.pow(((input - dDeadband) * (1/(1-dDeadband))), dSquareFactor);
    else if(input < 0) return ((Math.pow(((Math.abs(input) - dDeadband) * (1/(1-dDeadband))), dSquareFactor)) * -1);
    else return 0;
  }

  private double ThrottleScale(final double throttle,final double input) {
		return (JoystickDeadBand(input) * (1-(throttle*dThrottleFactor)));
	}

  //creates a driving function using specified joystick
  public void driveWithJoystick(final Joystick stick) {

    //creates variables for joystick x and y values
    final double zRotation = ThrottleScale(Math.abs(stick.getY()* 1), stick.getRawAxis(4)* -1);
    final double xSpeed = JoystickDeadBand(stick.getY()* 1);

    //uses joystick to do driving thing
    drRobotDrive.curvatureDrive(xSpeed, zRotation, true);

    // double dSquareFactor2 = SmartDashboard.getNumber("dSquareFactor", dSquareFactor);
    // double dThrottleFactor2 = SmartDashboard.getNumber("dThrottleFactor", dThrottleFactor);

    // if((dSquareFactor2 != dSquareFactor)) {dSquareFactor = dSquareFactor2;}
    // if((dThrottleFactor2 != dThrottleFactor)) {dThrottleFactor = dThrottleFactor2;}
  }

  //stops the drive train
  public void DriveStop() {
    drRobotDrive.arcadeDrive(0, 0);
  }

  public void DriveTank(final double leftValue, final double rightValue) {
    drRobotDrive.tankDrive(leftValue, rightValue);
  }

  //sets the speed for climbing drive thingy
  public void ClimbSpeed() {
    drRobotDrive.arcadeDrive(-0.7, 0);
  }

  //shifts drive train to low gear
  public void HighGear() {
    this.slndShift.set(false);
    bIsLow = false;
    SmartDashboard.putBoolean("Gear", bIsLow);
  }

  //shifts drive train to high gear
  public void LowGear() {
    this.slndShift.set(true);
    bIsLow = true;
    SmartDashboard.putBoolean("Gear", bIsLow);
  }

  //toggles gear state
  public void toggleGear() {
    if (bIsLow) {
      HighGear();
    } else {
      LowGear();
    }
    SmartDashboard.putBoolean("Gear", bIsLow);
  }

  //gets encoder position for the left side
  /*
  public Double getleftEncederValue() {
    Double enCode1 = enLeft1.getPosition();
    Double enCode2 = enLeft2.getPosition();
    if (enCode1 == enCode2){
      return enCode1 - enLeftStart;
    }else{
      System.out.println("Left encoder1:" + enCode1.toString() + " encoder2:" + enCode2.toString());
      return Math.max(enCode1, enCode2) - enLeftStart;
    }
  }
  */


  /*
  //gets the encoder position for the right side
  public Double getRightEncederValue() {
    //greenjamesag@gmail.com
    Double enCode1 = enRight1.getPosition();
    Double enCode2 = enRight2.getPosition();
    if (Math.abs(enCode1) == Math.abs(enCode2)){
      return enCode1 - enRightStart;
    }else{
      System.out.println("Left encoder1:" + enCode1.toString() + " encoder2:" + enCode2.toString());
      return Math.max(enCode1, enCode2) - enRightStart;
    }
  }
  */


  /*
  //resets encoders to current value
  public Boolean resetEncoder(){
    enLeftStart = getleftEncederValue();
    enRightStart = getRightEncederValue();
    return true;
  }
  */
}

/*
  
ooooooooooooooollllllllccccccccccccccccccccccccccclccccccccccccccccclclllllllllllllllllooooooooooooo
oooooooolllolloooolllllllllllllccccccccccc;''',:lddc;;ldlc:;,;:ccccccllllllllllllllllloooooooooooooo
ooooooollloooooooooolllllllllllllllccccl:'';:;,.:dd;..:kl'',,,'';ccccccccllllllllllllllooooooooddddd
oooooooollloolllllllllllllllcccccccccccc'';:clc;cxd;..:Ol,:lccc,'ccccccclllllllllloooooooooooooooood
looooooolllollllllllllllllllccccccccccc:..';ccclkxo:llcOOolllc:'':llllcllllllllllllooooooooooooooooo
llllllolllllllllllllllccccccccccccccccc:..';cloxOxccdl:kKOxol:,..:lllllllllllllllllooooooooooooooooo
llllllllllllllllllllcccccccccllllccccclc..''cxkOOo,;lc:k0OOkl,'..:lllllllllllllllloooooooooooooooooo
llllllllllllllllllllcccccccclllllllllclc'   .,cl,..,::,,co:,... .:lllllllllllllllooooooooooooooooooo
lllllllllllllcccccccccccllllllllllllcclc'     .'..';:c;.',......';;:cllllllloooolllloooloooooooooooo
lllllllllllllllllllcccclllllllllllllcclc'     ....,:loc..'.  .. ...'cllllloooooooooooolllooooooooooo
lllllllccccllccccccccclllllllllllclllllc'  ...;;..;:lo:.,c' ..  .clllloooooooooooooooooooodooooooooo
lllllllllllolllllllllllllllllllllllllllc' .,'.;;.':coo;.,c'.',. .:oooooooooooooooooooooooodooooooooo
oooooooollloollllllllllllllccclccl:;looo:,;cccl:,:llool;;c,.;c;.':clooolllloooooollllllllooooooolloo
llllllllllloollllllllllllllcccccc:,,;;ldocloc:coloxdol:codooocccoxxxdlc:;coooooooooooooooooooooooloo
lllllllllllollllllllllllllccccccc:::;:c:...,..:,.,l:::,',c,,;''',loocc:;,:loooooooooooooooollooooooo
lllllllllllllllllllllllllccccccccccc:cc:. .. .,'.,c:ll..':'...  .clccllcllloooooooooooooooollloooooo
lllllllllllllllllllllllllllllcccccll::c:.     .. ':cod;....     .;:cllcclllllloooooooooooooooooooooo
lllllllllllllcccccllllllllllllllclol::c:.     ,' 'cllo;..;.     .,clc::lolllllllllloooooooollooooooo
lllllllllllllcccccllllllllllllllclolc::;.    .;' 'codo;.':.     .;cc::cloollllllloooooooooollooooooo
llllllllllllllcccccccccccccclllcclolccc;.',.';;. .,,;;'.':,...,..;cccclooollllllooooooooooollllloooo
lllllllllllllllllllccccclllllllllloolll;.;:,;:c' .',::'.'cc;,:l'.:lccloooooooooooooooooooooooooooooo
llllllllllllollllllllllllllllllcllollll;',,'',;. .;:oo,.'::;,;:..:cclloooololllooooooooooooooooooooo
llllllllllcllccccllllllllcccccccclllccc;..     . .:okx,.........':clllloollllllloooooooolooooooooool
llllllllllclllllllllllllllllllclclllccc,.        .:lkk,...      ,cllllloollllllllllloooolooooooooooo
lllllllccccllllllllllllllllccccllllcccc,.  .     .:lxo'..      .;lllllloollllllllllllllllooooooooooo
llllllllllclllllllllccccccccclllloo;;cc,.        .:lxc. .    ...;llll:cdollllllllllllllllooooooooooo
lllllllllllllllllllllcclllllcllllooc;:c,..      .';:c;...    . .;lll::oddllllllllllllllllooloooooooo
llllllllllllllllllllcclllccccccclllcc;;...  .   .,;;::''.      .;lc::lodolllllllllllllllloollllllllo
llllllllllllllccccccccccccccccc::c:,,,'........'.,:;::,'........;:,;:clllllllllooloolllllllllllllloo
lcclllllclcllcccccccccc:cc:c:;,'',........  . ',,cl:,;,;,,;.....'.','',:clllollllllllllllllllllllloo
llllllllllllllccccccccccccccc'....,::c;...    ...,,..,,'........';:::,.',,;:llllollolllllooooooooooo
lllllllllllllcccccccccccccccc:,',:clclc'..      .,:;,::'... .. .,cllll:,'..,clllllllllllllolllllllll
llllllcccccclcccccccccccccccccccccccccc,.      ..';cdl;'...... .,:llllll:::cllllllllllllllolllllllll
llllllcccccclccccccccccccccccccccccccccc'.  ..''..':lc,.    ....;cllllllllllllllllllllllllllllllllll
llllccccccccclcccccccccccccc::::ccccccccc;..,c:...';:'.......'.,clllllllllllllllllllllllllllllllllll
lcccccccccccccccccccccccccc::::::::cccccccc'.,;,.,;c:..';c:...;cllllllllllllllllllccclllllllllllllll
cccccccccc:cccccccccccccccc:::::::::ccccccc;,..',..''..,,'..,:cccccccccclllllcccllllllllllllllllllll
cccccccccccccccccccccccccc::::::::::::::ccc;,..,;.  ...'...:ccccccccccccccccccllllllllllcllcccclllll
ccccccccccccccc:ccccccc::::::::::::::::::cc;,..,;...  ';..,cccccccccccccccccclllllllllllcllcccccllll
llccccccccccccccccccccc::::::::::::::::::::;;. ';.....,;..,clcl:,:lllllllllllcccclllllllllllllllllll
lcccccccccccllcccccccccccccc:::::::::;;:'';,,. .;,'...''...,;;:;;ldollcc:lllccccccccclccllllllllllll
cccccccccccccc::cc:::::::::ccccccc:::',:;;,'....''..  .........':lodl;,''lOxc:::cccccccccllcllllllll
;;;;;;;;;;;;;;;;;;;;;;;:cclllccccc::;;;::;;::,.',......'..''''.........,:,;'.'..::;;::::::::::::::::
,,,,,,,,,,,,,,,,,''''',;:cldkOkxdlc:::::;;;;;,';:,''',;c:;:;,'.........;'.......,;,;;;;;;;,,,,,;;;;;
,;,,;;,,;,,,,,,,,,,,,,;::coxkkkxdlc:;,,,;;;,,'',;,,,;;;:;,;,'..  .....''.''...''',,,,;ccccc:;;;;::::
xxxxxxxxxxdoolcc::;::;:cllc:::;;;,,,,,,,,;;,;;;;::ccc::::;,..............'.....',;'';:cllllllxkkkkkk
0000000Odolc:::;,'''''',;:::cl::c:,,,,'',,',,,,,;:cc;,;'..'..''''..';,,'.   .   .,,,;;:clcccoO0000KK
000000Ooccccccccc::;''.........',;,;::;;:;,;;,,,,;:c:,,,,'.,:'..,'.:l:,..  .....'..''',;:cccd00000KK
000000kc::cccllllllllcc::;,'...........'',;::::cccccll:;:,;lo,.''';lc,,.....''','..'',,;::clk00000KK
KK000Kkc;;;;;;:::cccldkxolllcc::;;,'............'',,;;;;:codo::,..cl;......,:;'....''',;:cdO0000KKKK
KKKK00Oo;;;,,;;,;;;;:oKKocxOOkdlllllllcc:;,,.... ........',;'..'...;;'',:cccc;',,...''',:ok0000KKKKK
KKK0000ko:;;;,;;,,,,,;kOc:dxkXOloO0Oxoloolllllcc:;;;,'''............,:lllooolc:;,''',,:lxO0000KKKKXX
KK000000OOxl;,'',,,,,:k0c,;cxKxlOKOKKolOK0Okdllllllllllllcccclc,'',;clooooolccldxkxkxxO0KKKKKKKXXXXX
KKKK00000Oxl:;'......,oxocddoOOoxd:lkdo0Oxkkl::cccllllloooollollclllcclllllcclk00000KKKKKKKKXXXXXXXX
KKKKKKKKK00OOxdoc:;'......':cdxcokdokxldxxkd:,;;;;;;:::cccllllolllooolccccc::lk00KKKKKKXXXXXXXXXXXXX
XXXXXXXXXXKKKKKK0OOkdol:;,'......;codccdockKd;,,,,,;;;;;;;;;:::ccccccc:::::::oOKKKKKKXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXKKKKK00Okxdolc;,'.....,:cddc,,,,;;,,,,,,,,,,,,;,,,;;;;:::cldO0KKKKKKXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXKKKKK00Oxdolc;,'........'',,,,,,,,,,,,,,,,,;;::::cx00000000KKXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXKKK00Okxdol:;,..........'',,,;;,,,,,;:::lxO000KKKKKXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXKKK0Okxdolc:;'........'''',,,:cdO0000KKKKXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXNXXXXXXXXXNNNNXXXXXXXXXXXXXXKKKK00Okxdoc:;,,'',,;;cdO000KKKKKKXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXNNXNNNNNNNNNNNNNNXXXXXXXXXXXXXXKKKK00OOkkxxxxk0KKKKKXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXNNNNNNNNNNNNNNNNNNNNNNNNNXXXXXXXXXXXXXXXXXXXXXXXXXXXXXNNNXXXXXXXXXXXXNNXXXXXX
XXXXXXXXXXXXXXXNNNNNXXXNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNXXXXXXXXXXXNNNNNNNNNNNNXXXXXNNNXXNNNNNNNNNXXXX











*/
