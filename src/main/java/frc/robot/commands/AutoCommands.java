package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auto_commands.Auto_Drive_Master;
import frc.robot.auto_commands.Auto_Finished;
import frc.robot.auto_commands.Auto_Intake_Off;
import frc.robot.auto_commands.Auto_Intake_On;
import frc.robot.auto_commands.Auto_Reset_Encoders;
import frc.robot.auto_commands.Auto_Shoot;
import frc.robot.auto_commands.Auto_Shooter_Warm_Up;
import frc.robot.auto_commands.Auto_Tank_Rotate_Master;
import frc.robot.auto_commands.Auto_Turn_To_Target_Master;
import frc.robot.auto_commands.Auto_Wait_Seconds;

/**
 *
 */
public class AutoCommands extends CommandGroup {

    public AutoCommands(int commandtorun) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        // Set Robot Color depending on Alliance
        /*
         * if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue)
         * { m_alliance = "BLUE"; Robot.rgbledCAN.LEDoff(); Robot.rgbledCAN.LEDblue(); }
         * else { m_alliance = "RED"; Robot.rgbledCAN.LEDoff();
         * Robot.rgbledCAN.LEDred(); }
         */

        // addSequential(new Auto_Reset_Encoders()); // reset drive encoders

        if (commandtorun == 1) { // 6 ball auto, straight lined up, our trench
          

            addSequential(new Auto_Reset_Encoders());
            addSequential(new Auto_Shooter_Warm_Up());            
            //addSequential(new Auto_Drive_Master(0, -40, 0.4, 0.2, 5, 5));
            //addSequential(new Auto_Wait_Seconds(0.3)); 
 

            //addSequential(new Auto_Turn_To_Target_Master());
            //addSequential(new Auto_Shoot(3));
            //addSequential(new Auto_Tank_Rotate_Master(0, 0.2, 0.2, 5));
            addSequential(new Auto_Intake_On());
            addSequential(new Auto_Drive_Master(0, -140, 0.5, 0.2, 10, 10));
            addSequential(new Auto_Wait_Seconds(0.8)); 

            // TODO: Go further back to get one or two more balls
//            addSequential(new Auto_Intake_Off());
//            addSequential(new Auto_Shooter_Warm_Up());
//            addSequential(new Auto_Drive_Master(0, 24, 0.4, 0.2, 10, 10));
            //addSequential(new Auto_Wait_Seconds(0.3)); 

            addSequential(new Auto_Intake_Off());

            addSequential(new Auto_Shooter_Warm_Up());

            addSequential(new Auto_Turn_To_Target_Master());
            addSequential(new Auto_Shoot(3));

            addSequential(new Auto_Intake_On());
            addSequential(new Auto_Tank_Rotate_Master(0, 0.2, 0.2, 5));
            addSequential(new Auto_Drive_Master(0, -92, 0.4, 0.2, 10, 10));
            addSequential(new Auto_Wait_Seconds(0.4)); 

            addSequential(new Auto_Intake_Off());
            addSequential(new Auto_Shooter_Warm_Up());
            addSequential(new Auto_Drive_Master(0, 92, 0.4, 0.2, 10, 10));
            addSequential(new Auto_Wait_Seconds(0.3)); 


            addSequential(new Auto_Turn_To_Target_Master());
            addSequential(new Auto_Shoot(3));




            addSequential(new Auto_Finished());

          
        } // command 1 done

        if (commandtorun == 2) { // 6 ball auto, target lined up, our trench
            addSequential(new Auto_Reset_Encoders());
            addSequential(new Auto_Shooter_Warm_Up());
            addSequential(new Auto_Turn_To_Target_Master());
            addSequential(new Auto_Shoot(3)); 
            addSequential(new Auto_Tank_Rotate_Master(-45, 0.2, 0.2, 5));


            addSequential(new Auto_Drive_Master(-45, -108, 0.4, 0.2, 10, 10));
            addSequential(new Auto_Tank_Rotate_Master(0, 0.2, 0.2, 5));
            addSequential(new Auto_Intake_On());
            addSequential(new Auto_Drive_Master(0, -72, 0.5, 0.2, 10, 10));
            addSequential(new Auto_Wait_Seconds(0.3)); 
            addSequential(new Auto_Intake_Off());
            

            addSequential(new Auto_Shooter_Warm_Up());
            addSequential(new Auto_Turn_To_Target_Master());
            addSequential(new Auto_Shoot(3)); 

            
            

            addSequential(new Auto_Finished());

        }

        if (commandtorun == 3){ //8 ball auto, opposite side steal, then generator


            /*
            addSequential(new Auto_Intake_On());
            addSequential(new Auto_Drive_Master(0, -120, 0.5, 0.2, 10, 10)); //Drive add pick up two from opposite trench
            addSequential(new Auto_Wait_Seconds(0.5)); //Wait
            addSequential(new Auto_Intake_Off());//Turn of intake
            addSequential(new Auto_Drive_Master(0, 24, 0.5, 0.2, 5, 5));
            addSequential(new Auto_Tank_Rotate_Master(45, 24, 0.5, 0.3, 10, 10)); //Curve so that parallel to generator
            addSequential(new Auto_Drive(45, 120, 0.7, 0.4, 10));// Drive parallel to generator
            addSequential(new Auto_Tank_Rotate_Master(45, 24, 0.5, 0.3, 10, 10));
            addSequential(new Auto_Shooter_Warm_Up());//Warm up shooter
            addSequential(new Auto_Turn_To_Target()); //Aim
            addSequential(new Auto_Shoot()); //Fire
            //addSequential(new Auto_Drive_Curve(45, 0, 0.5, 0.3, 10, 10)); //Turn back to angle
            //addSequential(new Auto_Drive_Curve(0, 24, 0.5, 0.3, 10, 10)); //Turn to 0
            addSequential(new Auto_Intake_On());// Intake on to pick up two from generator
            addSequential(new Auto_Drive(0, -12, 0.7, 0.4, 10));//Drive back
            addSequential(new Auto_Wait_Seconds(1)); //Wait
            addSequential(new Auto_Drive(0, 12, 0.7, 0.4, 10)); //Move forwards
            addSequential(new Auto_Drive_Curve(-60, -24, 0.5, 0.3, 10, 10)); //Curve to last ball
            addSequential(new Auto_Intake_Off()); //Intake off
            addSequential(new Auto_Shooter_Warm_Up()); //Warm up
            addSequential(new Auto_Drive_Curve(0, 48, 0.5, 0.3, 10, 10)); //Move forwards and get back to 0
            addSequential(new Auto_Turn_To_Target());// Aim
            addSequential(new Auto_Shoot()); //Fire
            addSequential(new Auto_Finished());
            */
            
            


        }

        if (commandtorun == 4){
            addSequential(new Auto_Reset_Encoders());
            addSequential(new Auto_Shooter_Warm_Up());
            addSequential(new Auto_Turn_To_Target_Master());
            addSequential(new Auto_Shoot(3)); 
            addSequential(new Auto_Tank_Rotate_Master(0, 0.2, 0.2, 5));
            addSequential(new Auto_Drive_Master(0, -36, 0.5, 0.2, 10, 10));            
            addSequential(new Auto_Finished());

        }


   

        

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
