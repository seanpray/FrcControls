// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
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

  Thread m_visionThread;

  private static final String kDefaultAuto = "blank";
  private static final String kPreload = "preload only";
  private static final String kPreloadBackup = "preload backup";
  private static final String kChargeDock = "charge dock";
  private String m_autoSelected;
  public static boolean auton = false;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final Timer m_timer = new Timer();

  public static boolean isAuton() {
    return auton;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(320, 240);
              camera.setFPS(3);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("video", 640, 480);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
               
                outputStream.putFrame(mat);
              }
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

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
    RobotContainer.drivetrain.resetGyro();
    auton = true;
    drive = false;
    m_timer.restart();
    // RobotContainer.elevator.resetEncoder();
    // RobotContainer.intake.resetEncoder();
    // RobotContainer.fourbar.resetEncoder();
    m_autoSelected = m_chooser.getSelected();
    System.out.println(m_autoSelected);
    // SmartDashboard.putData(m_chooser);
    RobotContainer.drivetrain.resetEncoders();
    // RobotContainer.elevator.pullUp(5);
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
      RobotContainer.intake.runIntakeIn(0.1);
      return true;
    }
    if (m_timer.get() < 4.0) {
      RobotContainer.intake.runIntakeOut(0.52);
      return true;
    }
    return false;
  }
// true when done
  boolean drive = false;
  // boolean drive_forward = false;
  // boolean turnaround = false;
  public boolean driveAuton(double power, double distance) {
    if (Math.abs(RobotContainer.drivetrain.encoderDistance()) < distance && !drive) {
      RobotContainer.drivetrain.holoDrive(power, 0);
      return false;
    }
    if (!drive) {
      RobotContainer.drivetrain.holoDrive(0, 0);
    }
    drive = true;
    // System.out.println(RobotContainer.drivetrain.getAngle());
    // if (!turnaround && Math.abs(Math.abs(RobotContainer.drivetrain.getAngle()) % 360 - 180) > 3) {
    //   boolean negative = RobotContainer.drivetrain.getAngle() < 0;
    //   double turnPower = (negative ? -1.2 : 1.2) * Math.abs(Math.abs(RobotContainer.drivetrain.getAngle()) % 360 - 180) / 180;
    //   if (turnPower > 0.5) {
    //     turnPower = 0.5;
    //   } else if (turnPower < -0.5) {
    //     turnPower = -0.5;
    //   }
    //   RobotContainer.drivetrain.holoDrive(0, turnPower * 0.6);
    //   return false;
    // }
    // if (!turnaround) {
    //   RobotContainer.drivetrain.holoDrive(0, 0);
    // }
    // turnaround = true;
    RobotContainer.drivetrain.setBrake(true);
    RobotContainer.drivetrain.setBrake(false);
    return true;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    RobotContainer.fourbar.run();
    switch (m_autoSelected) {
      case kPreloadBackup:
        if (preloadScore()) {
          return;
        }
        if (driveAuton(0.2, 120)) {
          break;
        }
        return;
      case kChargeDock:
        if (preloadScore()) {
          return;
        }
        if (driveAuton(0.2, 72)) {
          break;
        }
        // Put custom auto code here
        return;
        //blank
      case kDefaultAuto:
        break;
        // preload only
      default:
        if (preloadScore()) {
          return;
        }
    }
    if (auton) {
      auton = false;
    }
    // System.out.println(auton);

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
