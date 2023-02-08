// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}

  public void initDefaultCommand() {
    
  }

  // initialize intake motor controller from intake constant
  VictorSP intake = new VictorSP(Constants.intake);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intake.set(0);
  }

  /**
   * Drive the intake at a given power
   * @param power power to drive the intake at
   */
  public void driveIntake(double power) {
    intake.set(power);
  }

  /**
   * Defaults to setting the intake to half power
   */ 
  public void driveIntake() {
    intake.set(-1);
  }

  /**
   * Reverse the intake at half power
   */
  public void reverseIntake() {
    intake.set(1);
  }

  /**
   * Stops the intake
   */
  public void stopIntake() {
    intake.stopMotor();
  }
}
