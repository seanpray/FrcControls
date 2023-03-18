// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    
    // Drivebase motor controller index values
    // CAN
    public final static int l1 = 1;
    public final static int l2 = 4;
    public final static int r1 = 2;
    public final static int r2 = 3;

    // CAN
    public final static int elevator_neo = 5;
    public final static int intake_rotate = 7;
    public final static int intake_intake = 6;
    public final static int fourbar_neo = 8;
    // pwm
    public final static int bj_servo = 9;
    // DIO
    public final static int elevator_limit_switch = 8;

    // Flywheel and intake motor controller index values
    public final static int Flywheel = 1;
    public final static int intake = 0;
    // MUST BE UPDATED FOR NEW ROBOT ^^^^^^^^^^^^^^^ 

  public static final class DriveConstants {

    public static final double kTrackwidthMeters = 0.55245;
    // public static final DifferentialDriveKinematics kDriveKinematics =
        // new DifferentialDriveKinematics(kTrackwidthMeters);j
        

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // Old characterization values: UPDATE ASAP!!!!!!!!!!!!!
    public static final double ksVolts = 0.118;
    public static final double kvVoltSecondsPerMeter = 2.8141;
    public static final double kaVoltSecondsSquaredPerMeter = .2588;
    public static final double kPDriveVel = 3.284;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {

    // Max speed and acceleration values
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  // Joystick index values
  public static final int driver = 0;
  public static final int operator = 1;  


}

