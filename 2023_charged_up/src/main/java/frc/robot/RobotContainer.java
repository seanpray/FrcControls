// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.autoGroup;
import frc.robot.commands.drive;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
    public final static DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

    public final static Intake intake = new Intake();
    // public final Shooter shooter = new Shooter();
  
    public static OI oi = new OI();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    drivetrain.setDefaultCommand(
      new drive(drivetrain)
    );

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /**
     * Bind button index 4 
     */
    new JoystickButton(oi.driver, 5)
      .whileActiveContinuous(new InstantCommand(intake::driveIntake));
    
    new JoystickButton(oi.driver, 6)
      .whileActiveContinuous(new InstantCommand(intake::reverseIntake)); 

    // new JoystickButton(oi.driver, 1)
      // .whenPressed(new InstantCommand(shooter::startFlywheel, shooter));

    // new JoystickButton(oi.driver, 2)
      // .whenPressed(new InstantCommand(shooter::stopFlywheel, shooter));

  
    // new JoystickButton(oi.driver, 4)
    //   .whenPressed(new InstantCommand(climber::retract, climber));

    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  public void getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint =
    // new DifferentialDriveVoltageConstraint(
    //     new SimpleMotorFeedforward(
    //         DriveConstants.ksVolts,
    //         DriveConstants.kvVoltSecondsPerMeter,
    //         DriveConstants.kaVoltSecondsSquaredPerMeter),
    //     DriveConstants.kDriveKinematics,
    //     10);

    // // Create config for trajectory
    // TrajectoryConfig config =
    // new TrajectoryConfig(
    //         AutoConstants.kMaxSpeedMetersPerSecond,
    //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics)
    //     // Apply the voltage constraint
    //     .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory =
    // TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     List.of(new Translation2d(-1, 0), new Translation2d(-2, 0)),
    //     new Pose2d(-3, 0, new Rotation2d(0)),
    //     // Pass config
    //     config);

    // RamseteCommand ramseteCommand =
    // new RamseteCommand(
    //     exampleTrajectory,
    //     drivetrain::getPose,
    //     new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    //     new SimpleMotorFeedforward(
    //         DriveConstants.ksVolts,
    //         DriveConstants.kvVoltSecondsPerMeter,
    //         DriveConstants.kaVoltSecondsSquaredPerMeter),
    //     DriveConstants.kDriveKinematics,
    //     drivetrain::getWheelSpeeds,
    //     new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //     new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //     // RamseteCommand passes volts to the callback
    //     drivetrain::tankDriveVolts,
    //     drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    // drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.

    //return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));

    
    // return new SequentialCommandGroup(new autoGroup(intake, shooter), ramseteCommand);    
  }

}
