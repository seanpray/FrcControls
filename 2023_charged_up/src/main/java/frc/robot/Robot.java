// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private static final String kDefaultAuto = "blank";
  private static final String kPreload = "preload only";
  private static final String kPreloadBackup = "preload backup";
  private static final String kChargeDock = "charge dock";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final Timer m_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Blank Auto", kDefaultAuto);
    m_chooser.addOption("Preload Backup", kPreloadBackup);
    m_chooser.addOption("Preload charge dock", kChargeDock);
    m_chooser.addOption("Preload only", kPreload);
    SmartDashboard.putData("Auto choices", m_chooser);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_timer.start();
    RobotContainer.elevator.resetEncoder();
    RobotContainer.intake.resetEncoder();
    RobotContainer.fourbar.resetEncoder();
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
  }
  // false if done
  public boolean preloadScore() {
    if (m_timer.get() < 2.0) {
      RobotContainer.intake.set_angle(15);
      return true;
    }
    if (m_timer.get() < 2.7) {
      RobotContainer.intake.run_intake_in(0.1);
      return true;
    }
    if (m_timer.get() < 4.0) {
      RobotContainer.intake.run_intake_out(0.5);
      return true;
    }
    return false;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kPreloadBackup:
        if (preloadScore()) {
          return;
        }
        if (RobotContainer.drivetrain.encoderDistance() < 72) {
          RobotContainer.drivetrain.holoDrive(-0.3);
        }
        return;
      case kChargeDock:
      if (preloadScore()) {
        return;
      }
        if (RobotContainer.drivetrain.encoderDistance() < 72) {
          RobotContainer.drivetrain.holoDrive(-0.3);
        }
        // Put custom auto code here
        return;
        //blank
      case kDefaultAuto:
        break;
        // preload only
      default:
        preloadScore();
        break;
    }
    RobotContainer.drivetrain.clearPowers();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
