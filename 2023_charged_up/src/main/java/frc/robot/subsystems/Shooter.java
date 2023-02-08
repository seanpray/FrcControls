// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {  
  
  private WPI_TalonFX shooterTalon = new WPI_TalonFX(0);
  
  /** Creates a new Shooter. */
  public Shooter() {

    shooterTalon.configFactoryDefault();

    shooterTalon.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
		shooterTalon.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
    shooterTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
      Constants.kPIDLoopIdx, 
		  Constants.kTimeoutMs);
											

		/* Config the peak and nominal outputs */
		shooterTalon.configNominalOutputForward(0, Constants.kTimeoutMs);
		shooterTalon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		shooterTalon.configPeakOutputForward(1, Constants.kTimeoutMs);
		shooterTalon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		shooterTalon.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		shooterTalon.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		shooterTalon.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		shooterTalon.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // shooterTalon.setSensorPhase(true);
    
    
  }  

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    double velocity = shooterTalon.getSelectedSensorVelocity() / 2048.0 * 600.0;
    SmartDashboard.putNumber("shooterVelo", velocity);
  }

  /**
   * Start spinning the flywheel
   */
  public void startFlywheel(){

    // sets the flywheel to spin at 3000 units per 100 ms

    /**
    * Convert 3000 RPM to units / 100ms.
    * 2048 Units/Rev * 4000 RPM / 600 100ms/min in either direction:
    * velocity setpoint is in units/100ms
    */

    double velocity = 3000 * 2048.0 / 600.0;
  

    shooterTalon.set(ControlMode.Velocity, velocity);
  
  }

  /**
   * Stops the flywheel
   */
  public void stopFlywheel(){

    shooterTalon.stopMotor();

  }
}
