// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.MecanumDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Penetrator;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static final double ksVolts = 0.118;
    public static final double kvVoltSecondsPerMeter = 2.8141;
    public static final double kaVoltSecondsSquaredPerMeter = .2588;
    public static final double kPDriveVel = 3.284;
  // The robot's subsystems and commands are defined here...
    public final static DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

    public final static Intake intake = new Intake();

    public final static Penetrator penetrator = new Penetrator();
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
    new JoystickButton(oi.driver, 4)
      .whenPressed(new InstantCommand(intake::toggle));
    // new JoystickButton(oi.driver, 5)
    //   .whileActiveContinuous(new InstantCommand(intake::run_intake_in));
    new JoystickButton(oi.driver, 2)
      .whileActiveContinuous(new InstantCommand(penetrator::increase_angle));
    new JoystickButton(oi.driver, 3)
      .whileActiveContinuous(new InstantCommand(penetrator::decrease_angle));
    new JoystickButton(oi.driver, 4)
      .whileActiveContinuous(new InstantCommand(intake::run_intake_out));
    new JoystickButton(oi.driver, 5)
      .toggleWhenPressed(new InstantCommand(intake::flip_intake));
    // new JoystickButton(oi.driver, 6)
    //   .whileActiveContinuous(new InstantCommand(intake::toggle));
    
    // new JoystickButton(oi.driver, 6)
    //   .whenPressed(new InstantCommand(intake::toggleCompressor)); 

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
     // Create config for trajectory
     TrajectoryConfig config =
     new TrajectoryConfig(
             AutoConstants.kMaxSpeedMetersPerSecond,
             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
         // Add kinematics to ensure max speed is actually obeyed
         .setKinematics(DrivetrainSubsystem.m_drivetrain);

 // An example trajectory to follow.  All units in meters.
 Trajectory exampleTrajectory =
     TrajectoryGenerator.generateTrajectory(
         // Start at the origin facing the +X direction
         new Pose2d(0, 0, new Rotation2d(0)),
         // Pass through these two interior waypoints, making an 's' curve path
         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
         // End 3 meters straight ahead of where we started, facing forward
         new Pose2d(3, 0, new Rotation2d(0)),
         config);

//  MecanumControllerCommand mecanumControllerCommand =
//      new MecanumControllerCommand(
//          exampleTrajectory,
//          DrivetrainSubsystem::getPose,
//          DriveConstants.kFeedforward,
//          DriveConstants.kDriveKinematics,

//          // Position controllers
//          new PIDController(AutoConstants.kPXController, 0, 0),
//          new PIDController(AutoConstants.kPYController, 0, 0),
//          new ProfiledPIDController(
//              AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

//          // Needed for normalizing wheel speeds
//          AutoConstants.kMaxSpeedMetersPerSecond,

//          // Velocity PID's
//          new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
//          new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
//          new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
//          new PIDController(DriveConstants.kPRearRightVel, 0, 0),
//          m_robotDrive::getCurrentWheelSpeeds,
//          m_robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
//          m_robotDrive);

//  // Reset odometry to the starting pose of the trajectory.
//  m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

//  // Run path following command, then stop at the end.
//  return mecanumControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
//      // return new SequentialCommandGroup(new autoGroup(intake, shooter), ramseteCommand);    
  }

}
