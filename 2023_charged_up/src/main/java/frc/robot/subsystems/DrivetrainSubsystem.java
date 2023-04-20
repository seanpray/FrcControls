/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Robot;

public class DrivetrainSubsystem extends SubsystemBase {  /**
   *
   * Creates a new DrivetrainSubsystem.
   */
  
  public void initDefaultCommand() {}
  public double gyroOffset = 0;

  /*
  Initialize drivebase motors from constants
  // */
  public static CANSparkMax frontLeft = new CANSparkMax(Constants.l1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  public static CANSparkMax backLeft = new CANSparkMax(Constants.l2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  public CANSparkMax backRight = new CANSparkMax(Constants.r2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  public CANSparkMax frontRight = new CANSparkMax(Constants.r1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  // Encoder frontLeftEncoder = new Encoder(0, 1, true, CounterBase.EncodingType.k4X);
  // Encoder frontRightEncoder = new Encoder(2, 3, true, CounterBase.EncodingType.k4X);
  // Encoder backRightEncoder = new Encoder(4, 5, true, CounterBase.EncodingType.k4X);
  // Encoder backLeftEncoder= new Encoder(6, 7, true, CounterBase.EncodingType.k4X);

  static Translation2d frontLeftLocation = new Translation2d(0.2286, 0.2286);
  static Translation2d frontRightLocation = new Translation2d(0.2286, -0.2286);
  static Translation2d backLeftLocation = new Translation2d(0.2286, -0.2286);
  static Translation2d backRightLocation = new Translation2d(-0.2286, -0.2286);
  public static MecanumDriveKinematics m_drivetrain = new MecanumDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);


  // public static MecanumDrive drive = new MecanumDrive((MotorController) DrivetrainSubsystem.frontLeft, (MotorController) DrivetrainSubsystem.backLeft, (MotorController) DrivetrainSubsystem.frontRight, (MotorController) DrivetrainSubsystem.backRight);


  
  // 12 60 -> 35 60 8 inch mecanum
  double encoderConstant = (1 / 8.57143) * 8 * Math.PI;

  // create a drivetrain from the backLeft and frontRight motors
  
  // initialize gyro
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  private final double curve_b = 0.1;
  
  // create odometry object to keep track of robot position
  static MecanumDriveOdometry m_odometry;

  double previousTimestamp = 0;
  double totalX = 0;
  double previousV = 0;
  public void poseApproximation() {
    double dt = (System.currentTimeMillis() - previousTimestamp) / 1000;
    // y is vertical, z is forward, x is sideways
    totalX += 0.5 * gyro.getAccelZ() * (dt * dt);
    // SmartDashboard.putNumber("X", Math.acos(gyro.getAccelZ() / 9.8065));
    // SmartDashboard.putNumber("Y", gyro.getAccelY());
    // SmartDashboard.putNumber("Z", gyro.getAccelZ());
    // SmartDashboard.putNumber("X3", gyro.getXFilteredAccelAngle());
    // SmartDashboard.putNumber("Y3", gyro.getYFilteredAccelAngle());
    // Math.atan(dt)

    // SmartDashboard.putNumber("encoder", encoderDistance());
    // System.out.println("encoder" + encoderDistance());
    // System.out.println("gyro " + totalX);
    // SmartDashboard.putNumber("gyro", totalX);

  }

  public void resetGyro() {
    gyroOffset = gyro.getAngle();
  }

  public DrivetrainSubsystem() {
    gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kY);
    gyro.configCalTime(edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime._256ms);
    // restores factory defaults on Spark MAX motor controllers and sets encoder positions to 0
    frontLeft.restoreFactoryDefaults();
    backLeft.restoreFactoryDefaults();
    frontRight.restoreFactoryDefaults();
    backRight.restoreFactoryDefaults();
    resetEncoders();
    frontLeft.getEncoder().setPositionConversionFactor(encoderConstant);
    frontRight.getEncoder().setPositionConversionFactor(encoderConstant);
    backRight.getEncoder().setPositionConversionFactor(encoderConstant);
    backLeft.getEncoder().setPositionConversionFactor(encoderConstant);
    frontLeft.setIdleMode(IdleMode.kCoast);
    frontRight.setIdleMode(IdleMode.kCoast);
    backRight.setIdleMode(IdleMode.kCoast);
    backLeft.setIdleMode(IdleMode.kCoast);


    // Sets the distance per pulse for the encoders to translate from encoder ticks to meters
    // frontRight.getEncoder().setPositionConversionFactor(encoderConstant);
  
  } 

  public final double dead_zone = 0.09;

   // can think of curve_b as shifting down so the graph so it starts at 0, basically applies a deadzone
  public static double curveInput(double v, boolean turbo, double curve_b) {
    var negative = v < 0.0;
    var c = (Math.sqrt(Math.abs(v)) - curve_b);
    if (c < 0) {
        c = 0;
    }
    return negative ? -c: c;
  }
  public static double clamp(double v, double l, double u) {
    if (v > u) {
      return u;
    } else if (v < l) {
      return l;
    } else {
      return v;
    }
  }

  public void setBrake(boolean state) {
    if (state) {
      frontLeft.setIdleMode(IdleMode.kBrake);
      frontRight.setIdleMode(IdleMode.kBrake);
      backRight.setIdleMode(IdleMode.kBrake);
      backLeft.setIdleMode(IdleMode.kBrake);
    } else {
      frontLeft.setIdleMode(IdleMode.kCoast);
      frontRight.setIdleMode(IdleMode.kCoast);
      backRight.setIdleMode(IdleMode.kCoast);
      backLeft.setIdleMode(IdleMode.kCoast);
    }
  }

  // in rad
  public double getHeading() {
    // double corrected_heading = gyro.getAngle();
    double corrected_heading = gyro.getAngle() - 180;
    // System.out.println("heading" + corrected_heading);
    boolean negative = corrected_heading < 0;
    corrected_heading = Math.abs(corrected_heading);
    double reference = corrected_heading % 360;
    if (negative) {
      reference = -reference;
    }

    return reference * Math.PI / 180.0;
  }

  @Override
  public void periodic() {
    // poseApproximation();
    // previousTimestamp = System.currentTimeMillis();
    SmartDashboard.putNumber("gyro angle", getAngle());
      
    

  
  }

  /**
   * Gets the current pose (2d position + heading) of the robot
   * @return current pose of the robot (meters)
   */
  public static Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  /**
   * Gets the differential wheel speeds of the drivebase
   * @return a DifferentialDriveWheelSpeeds object with the left and right wheel velocity
   */
  // public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  // }



  /**
   * Get the turning rate of the robot from the gyro
   * @return Turning rate of the robot (degrees per second)
   */
  public double getTurnRate() {
    return gyro.getRate();
  }

  /**
   * Resets the used encoders to 0
   */
  public void resetEncoders() {
    backLeft.getEncoder().setPosition(0);
    frontRight.getEncoder().setPosition(0);
    backRight.getEncoder().setPosition(0);
    frontLeft.getEncoder().setPosition(0);
  }

  // only useful for going in straight lines
  public double encoderDistance() {
    return (frontLeft.getEncoder().getPosition() + frontRight.getEncoder().getPosition() + backRight.getEncoder().getPosition() + backLeft.getEncoder().getPosition()) / 4;
  }
  public void setPower(double frontLeftPower, double backRightPower, double frontRightPower, double backLeftPower) {
    frontLeft.set(frontLeftPower * PID.motor_1);
    backLeft.set(backLeftPower * PID.motor_4);
    frontRight.set(frontRightPower * PID.motor_3);
    backRight.set(backRightPower * PID.motor_2);
  }
  public double getAngle() {
    return gyro.getAngle();
  }
  // distance in feet
  public void holoDrive(double power, double rx) {
    double botHeading = getHeading();
    frontRight.setInverted(true);
    backRight.setInverted(true);
    double rotX = power * Math.sin(botHeading);
    double rotY = power * Math.cos(botHeading);
    
    double y = -power; // Remember, this is reversed!
    setPower(rotY + rotX + rx, rotY - rotX - rx, rotY + rotX - rx, rotY - rotX + rx);
  }
}