// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Penetrator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBar;


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
    public final static FourBar fourbar = new FourBar();

    public final static Penetrator penetrator = new Penetrator();
    public final static Elevator elevator = new Elevator();
    // public final Shooter shooter = new Shooter();
  
    public static OI oi = new OI();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
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
    // new JoystickButton(oi.driver, 1)
      // toggleWhenPressed(new InstantCommand(penetrator::score));
    new JoystickButton(oi.driver, 4)
      .whenPressed(new InstantCommand(intake::toggle));
    new JoystickButton(oi.operator, 4)
      .whenPressed(new InstantCommand(intake::toggle));
    // servo in out
    new JoystickButton(oi.driver, 2)
      .toggleWhenPressed(new InstantCommand(penetrator::toggle));
    new JoystickButton(oi.operator, 2)
      .toggleWhenPressed(new InstantCommand(penetrator::toggle));

    // new JoystickButton(oi.driver, 4)
      // .whileActiveContinuous(new InstantCommand(intake::run_intake_out));
      
    new JoystickButton(oi.driver, 5)
      .toggleWhenPressed(new InstantCommand(intake::setIntakeLevel));
    new JoystickButton(oi.operator, 5)
      .toggleWhenPressed(new InstantCommand(intake::setIntakeLevel));
    new JoystickButton(oi.operator, 6)
      .toggleWhenPressed(new InstantCommand(elevator::elevatorPreset));

    }

}