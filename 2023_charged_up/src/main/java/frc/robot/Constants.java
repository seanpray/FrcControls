// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
    // public final static int l1 = 1;
    // public final static int l2 = 4;
    // public final static int r1 = 2;
    // public final static int r2 = 3;
    public final static int l1 = 3;
    public final static int l2 = 2;
    public final static int r1 = 4;
    public final static int r2 = 1;

    // Flywheel and intake motor controller index values
    public final static int Flywheel = 1;
    public final static int intake = 0;
    // MUST BE UPDATED FOR NEW ROBOT ^^^^^^^^^^^^^^^ 

  public static final class DriveConstants {

    public static final double kTrackwidthMeters = 0.55245;
    // public static final DifferentialDriveKinematics kDriveKinematics =
        // new DifferentialDriveKinematics(kTrackwidthMeters);

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


  	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon FX supports multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    public static final int kTimeoutMs = 30;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
     * 
	 * 	                                    			  kP   	 kI    kD      kF          Iz    PeakOut */
   public final static Gains kGains_Velocit  = new Gains( 0.1, 0.001, 5, 1023.0/20660.0,  300,  1.00);


   // ADJUST THESE VALUES FOR OUR ROBOT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

}

