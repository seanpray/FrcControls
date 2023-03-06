// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class autoGroup extends SequentialCommandGroup {

  Shooter m_shooter;
  Intake m_intake;

  /** Add your docs here. */
  public autoGroup(Intake intake, Shooter shooter) {

   m_shooter = shooter;
   m_intake = intake;

  addCommands(

    // start flywheel
    // new InstantCommand(m_shooter::startFlywheel, m_shooter),
    // wait 5 seconds to let flywheel spin up
    // new WaitCommand(5),
    // start the intake to feed the preloaded ball to the shooter
    // new InstantCommand(m_intake::driveIntake, m_intake),
    // wait 1 second to let the ball feed
    // new WaitCommand(1),
    // stop the intake
    // new InstantCommand(m_intake::stopIntake, m_intake)
  );
  }
}
