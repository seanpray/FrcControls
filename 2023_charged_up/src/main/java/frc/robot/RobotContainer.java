// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.pickup;
import frc.robot.commands.rotate;
import frc.robot.commands.rotateback;
import frc.robot.commands.setIntakeHalf;
import frc.robot.commands.setIntakeIn;
import frc.robot.commands.setIntakeOut;
import frc.robot.commands.taxi;
import frc.robot.commands.taxiback;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PID;
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

    intake.setDefaultCommand(new RunCommand(() -> {
      // SmartDashboard.putBoolean("switch", intakeSwitch.get());
        if (RobotContainer.oi.driver.getAButton() || RobotContainer.oi.operator.getAButton()) {
            intake.angle -= 0.3;
            // if (intakeSwitch.get()) {
            //     rotateEncoder.setPosition(-80);
            //     level = 0;
            // } else {
            //     angle -= 0.5;
            //     level = -1;
            // }   
        }
        if (RobotContainer.oi.driver.getBackButton() || RobotContainer.oi.operator.getBackButton()) {
            intake.angle += 0.5;
            intake.level = -1;
        }
        // double outtake = 0;
        
        double outtake = RobotContainer.oi.driver.getLeftTriggerAxis();
        double intake_power = outtake > 0.1 ? outtake < 0.1 ? -0.1 : -outtake * 0.33 : RobotContainer.oi.driver.getRightTriggerAxis() > 0.1 ? 0.52 : 0;
        if (RobotContainer.oi.driver.getStartButton()) {
            intake_power = 0.32; //mid power
            //intake = 1;  //full power
        } else if (RobotContainer.oi.driver.getRightBumper()) {
            intake_power = 0.1;
        } 
        if (Math.abs(intake_power) >= 0.1) {
            intake.runIntakeOut(intake_power);
        } else {
            intake.runIntakeIn(0.05);
        //    runIntakeIn(0);
        }

      
        // System.out.println(auton);
        // System.out.println();
        // SmartDashboard.putNumber("rotate", rotateEncoder.getPosition());
    }, intake));

    drivetrain.setDefaultCommand(new RunCommand(()->{
          //brake = RobotContainer.oi.driver.getRightStickButton() || RobotContainer.oi.operator.getRightStickButton();
    drivetrain.frontRight.setInverted(true);
    drivetrain.backRight.setInverted(true);
    
    //   var wheelPositions = new MecanumDriveWheelPositions(
    //   frontLeft.getEncoder().getPosition(), frontRight.getEncoder().getPosition(),
    //   backLeft.getEncoder().getPosition(), backRight.getEncoder().getPosition());

    // // Get the rotation of the robot from the gyro.
    // var gyroAngle = gyro.getAngle();
    // System.out.println(gyroAngle);

    // Update the pose
    double y = -RobotContainer.oi.driver.getLeftY(); // Remember, this is reversed!
    double x = -RobotContainer.oi.driver.getLeftX(); // Counteract imperfect strafing
    double rx = -RobotContainer.oi.driver.getRightX();
    final double dead_zone = 0.09;
    final double curve_b = 0.1;

    
    if (y < dead_zone && y > -dead_zone) {
      y = 0;
    }
    if (x < dead_zone && x > -dead_zone) {
      x = 0;
    }
    if (rx < dead_zone && rx > -dead_zone) {
      rx = 0;
    }
    if (rx == 0) {
      rx = -RobotContainer.oi.operator.getRightX();
    }
    if (rx < dead_zone && rx > -dead_zone) {
      rx = 0;
    }
    rx *= 0.7;

    double multiplier = 0.35;
    if (RobotContainer.oi.driver.getLeftStickButton()) {
      multiplier = 1;
    } 
    double botHeading = drivetrain.getHeading();
   
    if (RobotContainer.oi.driver.getXButton()) {
      drivetrain.gyroOffset = botHeading;
    }
    botHeading -= drivetrain.gyroOffset;
    // m_odometry.update(new Rotation2d(corrected_heading), new MecanumDriveWheelSpeeds(frontLeft.getEncoder().getVelocity(), frontRight.getEncoder().getVelocity(), backLeft.getEncoder().getVelocity(), backRight.getEncoder().getVelocity()));

    double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
    double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
    double frontLeftPower = (rotY + rotX + rx) / denominator;
    double backLeftPower = (rotY - rotX + rx) / denominator;
    double frontRightPower = (rotY - rotX - rx) / denominator;
    double backRightPower = (rotY + rotX - rx) / denominator;
    drivetrain.setPower(drivetrain.clamp(drivetrain.curveInput(PID.motor_1 * frontLeftPower * multiplier, multiplier == 1, curve_b), -1, 1), drivetrain.clamp(drivetrain.curveInput(PID.motor_2 * backRightPower * multiplier, multiplier == 1, curve_b), -1, 1), drivetrain.clamp(drivetrain.curveInput(PID.motor_3 * frontRightPower * multiplier, multiplier == 1, curve_b), -1, 1), drivetrain.clamp(drivetrain.curveInput(PID.motor_4 * backLeftPower * multiplier, multiplier == 1, curve_b), -1, 1));
    }, drivetrain));

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
      .onTrue(new InstantCommand(intake::toggle));
    new JoystickButton(oi.operator, 4)
      .onTrue(new InstantCommand(intake::toggle));
    // servo in out
    new JoystickButton(oi.driver, 2)
      .toggleOnTrue(new InstantCommand(penetrator::toggle));
    new JoystickButton(oi.operator, 2)
      .toggleOnTrue(new InstantCommand(penetrator::toggle));

    // new JoystickButton(oi.driver, 4)
      // .whileActiveContinuous(new InstantCommand(intake::run_intake_out));
      
    new JoystickButton(oi.driver, 5)
      .toggleOnTrue(new InstantCommand(intake::setIntakeLevel));
    new JoystickButton(oi.operator, 5)
      .toggleOnTrue(new InstantCommand(intake::setIntakeLevel));
    new JoystickButton(oi.operator, 6)
      .toggleOnTrue(new InstantCommand(elevator::elevatorPreset));

    }

  private static final String kDefaultAuto = "blank";
  private static final String kPreload = "preload only";
  private static final String kPreloadBackup = "preload backup";
  private static final String kPreloadBackupBump = "preload backup bump";
  private static final String kChargeDock = "charge dock";

  public Command getAutonomousCommand(String m_autoSelected) {

    
    switch (m_autoSelected){
      case(kDefaultAuto):
        return new InstantCommand();
      case(kPreloadBackup):
        return new SequentialCommandGroup(

          new InstantCommand(()->{intake.runIntakeIn(0.05);}),
          new setIntakeOut(intake),
          new WaitCommand(.5),
          new RunCommand(()->{intake.runIntakeOut(.52);}).withTimeout(1),
          new WaitCommand(.5),
          //new setIntakeIn(intake),
          new taxi(drivetrain).withTimeout(1.5),
          new rotate(drivetrain)
          
        );
        case(kPreloadBackupBump):
        return new SequentialCommandGroup(

          new InstantCommand(()->{intake.runIntakeIn(0.05);}),
          new setIntakeOut(intake),
          new WaitCommand(.25),
          new RunCommand(()->{intake.runIntakeOut(.52);}).withTimeout(1),
          new WaitCommand(.5),
          //new setIntakeIn(intake),
          new taxi(drivetrain).withTimeout(1.8),
          new rotate(drivetrain),
          new setIntakeIn(intake),
          new InstantCommand(()->{intake.runIntakeIn(.5);}),
          new pickup(drivetrain).withTimeout(0.85),
          new InstantCommand(()->{intake.toggle();}),
          new WaitCommand(.125),
          new InstantCommand(()->{intake.runIntakeIn(0.05);}),
          new setIntakeOut(intake),
          new rotateback(drivetrain),
          new taxiback(drivetrain).withTimeout(1.25),
          new setIntakeHalf(intake).withTimeout(.5),
          new RunCommand(()->{
            System.out.print("shooting cone");
            intake.runIntakeOut(1);
          }).withTimeout(0.25),
          new InstantCommand(()->{intake.runIntakeIn(0.05);})
        );
      case(kPreload):
        return new SequentialCommandGroup(
          new InstantCommand(()->{intake.runIntakeIn(0.05);}),
          new setIntakeOut(intake),
          new WaitCommand(.5),
          new RunCommand(()->{intake.runIntakeOut(.52);}).withTimeout(1),
          new WaitCommand(.5),
          new setIntakeIn(intake)
        );
      case(kChargeDock):
        return new SequentialCommandGroup(

      );

    }

    return new InstantCommand();

  }

}