// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class rotate extends CommandBase {
  
  DrivetrainSubsystem drivetrain;

  public static double clamp(double v, double l, double u) {
    if (v > u) {
      return u;
    } else if (v < l) {
      return l;
    } else {
      return v;
    }
  }


  /** Creates a new rotate. */
  public rotate(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  double currentAngle;
  double targetAngle;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.holoDrive(0, clamp(Math.abs(drivetrain.getAngle() - 180) / 180 * 0.45, .11, 1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.holoDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drivetrain.getAngle()-180) < 5.0;
  }
}
