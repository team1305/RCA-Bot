/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;

/**
 * The Shooter launches "Power Cells" from the robot to the "Power Port"
 */
public class ShooterRPM extends SubsystemBase {

    private final WPI_TalonFX shooterA = RobotMap.mtShootLeft1;
    private final WPI_TalonFX shooterB = RobotMap.mtShootRight1;

    public ShooterRPM() {

        
        
        shooterA.configFactoryDefault();
        shooterB.configFactoryDefault();
        shooterA.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);


        shooterA.setInverted(false);
        shooterA.setSensorPhase(true);

        shooterB.setInverted(true);
        shooterB.setSensorPhase(true);

        shooterA.setNeutralMode(NeutralMode.Coast);
        shooterB.setNeutralMode(NeutralMode.Coast);

//        shooterA.configClosedLoopPeakOutput(kControlSlot, Constants.kShooterMaxPrecentOutput);

        shooterA.config_kP(0, RobotMap.kP_SHOOTER);
        shooterA.config_kI(0, RobotMap.kI_SHOOTER);
        shooterA.config_kD(0, RobotMap.kD_SHOOTER);
        shooterA.config_kF(0, RobotMap.kF_SHOOTER);
        shooterA.config_IntegralZone(0, RobotMap.kIZone_SHOOTER);

        shooterA.clearStickyFaults();
        shooterB.clearStickyFaults();

        shooterB.follow(shooterA);
    }


    public void setShooterSpeed(double speed) {
        shooterA.set(ControlMode.PercentOutput, speed);
    }

    public void resetShooterPosition() {
        shooterA.setSelectedSensorPosition(0);
    }

    public double getShooterRotations() {
        return shooterA.getSelectedSensorPosition() / RobotMap.SHOOTER_OUTPUT_TO_ENCODER_RATIO / RobotMap.TICKS_PER_ROTATION;
    }

    public double getShooterRPM() {
        return shooterA.getSelectedSensorVelocity() / RobotMap.SHOOTER_OUTPUT_TO_ENCODER_RATIO / RobotMap.TICKS_PER_ROTATION * 10.0 * 60.0;
    }

    public void setShooterRPM(double rpm) {
        // double kF = (shooterA.getMotorOutputPercent() * 1023) / shooterA.getSelectedSensorVelocity();
        // shooterA.config_kF(0, kF);
        shooterA.set(ControlMode.Velocity, shooterRPMToNativeUnits(rpm));
    }

    public double shooterRPMToNativeUnits(double rpm) {
        return rpm * RobotMap.SHOOTER_OUTPUT_TO_ENCODER_RATIO * RobotMap.TICKS_PER_ROTATION / 10.0 / 60.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Rotations", getShooterRotations());
        SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooter Output Percent", shooterA.getMotorOutputPercent());
        // SmartDashboard.putNumber("Shooter Velocity Native", shooterA.getSelectedSensorVelocity());
        // SmartDashboard.putNumber("Shooter Stator Current", shooterA.getStatorCurrent());
        // SmartDashboard.putNumber("Shooter Supply Current", shooterA.getSupplyCurrent());
     }


	public void set(ControlMode percentoutput, double d) {
	}
}
