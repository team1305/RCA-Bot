/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	/*
	public static final int FRONT_LEFT_DRIVE_CAN = 0;
	public static final int MID_LEFT_DRIVE_CAN = 1;
	public static final int REAR_LEFT_DRIVE_CAN = 2;
	public static final int FRONT_RIGHT_DRIVE_CAN = 3;
	public static final int MID_RIGHT_DRIVE_CAN = 4;
	public static final int REAR_RIGHT_DRIVE_CAN = 5;
	public static final int CONVEYOR_CAN_ID = 6;
	public static final int INTAKE_CAN_ID = 7;
	public static final int SHOOTER_LEFT_CAN_ID = 9;
	public static final int SHOOTER_RIGHT_CAN_ID = 10;
	
	public static final int INTAKE_SOLENOID_FORWARD_PORT = 0;
	public static final int INTAKE_SOLENOID_REVERSE_PORT = 1;
	public static final int GATE_SOLENOID_PORT = 2;
	public static final int ARM_SOLENOID_PORT = 3;

	public static final int LEFT_DRIVE_ENCODER_PORT_1 = 0;
	public static final int LEFT_DRIVE_ENCODER_PORT_2 = 1;
	public static final int RIGHT_DRIVE_ENCODER_PORT_1 = 2;
	public static final int RIGHT_DRIVE_ENCODER_PORT_2 = 3;

	public static final int INTAKE_CELL_COUNTER_PORT = 0;
	public static final int SHOOTER_CELL_COUNTER_PORT = 1;

	public static final int DRIVER_CONTROLLER_PORT = 0;
	public static final int OPERATOR_CONTROLLER_PORT = 0;

	public static final int LED_PWM_PORT = 0;
	*/
	public static final double DEADBAND = 0.1;

	public static final double ROTATE_KP = 0.3; // arbitrary
	public static final double ANGLE_THRESHOLD = 0.5;
	public static final double DRIVE_KP = 0.3; // arbitrary
	public static final double FEET_PER_ROTATIONS = 1.5; // arbitrary



	public static final boolean GYRO_REVERSED = true;


	// PATH PLANNING
	 // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
	// values for your robot.
	

    public static final double ksVolts = 0.22; //CHANGE THIS
    public static final double kvVoltSecondsPerMeter = 1.98; //CHANGE THIS
    public static final double kaVoltSecondsSquaredPerMeter = 0.2; //CHANGE THIS

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;
	public static final double kTrackwidthMeters = 0.4955; //CHANGE THIS
    public static final DifferentialDriveKinematics kDriveKinematics =
		new DifferentialDriveKinematics(kTrackwidthMeters);
	public static final double kMaxSpeedMetersPerSecond = 3; //CHANGE THIS
	public static final double kMaxAccelerationMetersPerSecondSquared = 3; //CHANGE THIS
	// Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
	public static final double kRamseteZeta = 0.7;
	




}