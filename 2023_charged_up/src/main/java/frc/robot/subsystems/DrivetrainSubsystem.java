/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {  /**
   *
   * Creates a new DrivetrainSubsystem.
   */
  
  public void initDefaultCommand() {}
  private double gyro_offset = 0;
  /*
  Initialize drivebase motors from constants
  // */
  public static CANSparkMax leftFront = new CANSparkMax(Constants.l1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  public static CANSparkMax leftBack = new CANSparkMax(Constants.l2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  public static CANSparkMax rightBack = new CANSparkMax(Constants.r2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  public static CANSparkMax rightFront = new CANSparkMax(Constants.r1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  // Encoder leftFrontEncoder = new Encoder(0, 1, true, CounterBase.EncodingType.k4X);
  // Encoder rightFrontEncoder = new Encoder(2, 3, true, CounterBase.EncodingType.k4X);
  // Encoder rightBackEncoder = new Encoder(4, 5, true, CounterBase.EncodingType.k4X);
  // Encoder leftBackEncoder= new Encoder(6, 7, true, CounterBase.EncodingType.k4X);

  static Translation2d frontLeftLocation = new Translation2d(0.2286, 0.2286);
  static Translation2d frontRightLocation = new Translation2d(0.2286, -0.2286);
  static Translation2d backLeftLocation = new Translation2d(0.2286, -0.2286);
  static Translation2d backRightLocation = new Translation2d(-0.2286, -0.2286);
  public static MecanumDriveKinematics m_drivetrain = new MecanumDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  // public static MecanumDrive drive = new MecanumDrive((MotorController) DrivetrainSubsystem.leftFront, (MotorController) DrivetrainSubsystem.leftBack, (MotorController) DrivetrainSubsystem.rightFront, (MotorController) DrivetrainSubsystem.rightBack);


  
  // 12 60 -> 35 60 8 inch mecanum
  double encoderConstant = (1 / 8.57143) * 8 * Math.PI;

  // add entry to network table to put robot X and Y values from odometry
  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

  // create field object to update robot position
  Field2d field = new Field2d();

  // create a drivetrain from the leftBack and rightFront motors
  
  // initialize gyro
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  private final double curve_b = 0.1;
  
  // create odometry object to keep track of robot position
  static MecanumDriveOdometry m_odometry;


  public DrivetrainSubsystem() {
    gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kY);
    gyro.configCalTime(edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime._256ms);
    // restores factory defaults on Spark MAX motor controllers and sets encoder positions to 0
    leftFront.restoreFactoryDefaults();
    leftBack.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();
    rightBack.restoreFactoryDefaults();
    resetEncoders();
    leftFront.getEncoder().setPositionConversionFactor(encoderConstant);
    rightFront.getEncoder().setPositionConversionFactor(encoderConstant);
    rightBack.getEncoder().setPositionConversionFactor(encoderConstant);
    leftBack.getEncoder().setPositionConversionFactor(encoderConstant);


    // Sets the distance per pulse for the encoders to translate from encoder ticks to meters
    // rightFront.getEncoder().setPositionConversionFactor(encoderConstant);

    SmartDashboard.putData("Field", field);

  
} 
  /**
   * Gets heading from gyro
   * @return heading of gyro in degrees
   */
  public double getHeading() {
    return gyro.getAngle();
  }

  public final double dead_zone = 0.08;

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

  @Override
  public void periodic() {
    rightFront.setInverted(true);
    rightBack.setInverted(true);
    
    //   var wheelPositions = new MecanumDriveWheelPositions(
    //   leftFront.getEncoder().getPosition(), rightFront.getEncoder().getPosition(),
    //   leftBack.getEncoder().getPosition(), rightBack.getEncoder().getPosition());

    // // Get the rotation of the robot from the gyro.
    // var gyroAngle = gyro.getAngle();
    // System.out.println(gyroAngle);

    // Update the pose
    double y = -RobotContainer.oi.driver.getLeftY(); // Remember, this is reversed!
    double x = -RobotContainer.oi.driver.getLeftX(); // Counteract imperfect strafing
    double rx = -RobotContainer.oi.driver.getRightX();
    if (y < dead_zone && y > -dead_zone) {
      y = 0;
    }
    if (x < dead_zone && x > -dead_zone) {
      x = 0;
    }
    if (rx < dead_zone && rx > -dead_zone) {
      rx = 0;
    }
    rx *= 0.5;

    double multiplier = 0.5;
    if (RobotContainer.oi.driver.getLeftStickButton()) {
      multiplier = 1;
    }
    
    if (multiplier != 1) {
      x /= Math.sqrt(2);
    }
   


    //  double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    //  double frontLeftPower = (y + x + rx) / denominator;
    //  double backLeftPower = (y - x + rx) / denominator;
    //  double frontRightPower = (y - x - rx) / denominator;
    //  double backRightPower = (y + x - rx) / denominator;

    // leftFront.set(clamp(curve(PID.motor_1 * frontLeftPower * multiplier, multiplier == 1), -1, 1));
    // rightFront.set(clamp(curve(PID.motor_2 * frontRightPower * multiplier, multiplier == 1), -1, 1));
    // rightBack.set(clamp(curve(PID.motor_3 * backRightPower * multiplier, multiplier == 1), -1, 1));
    // leftBack.set(clamp(curve(PID.motor_4 * backLeftPower * multiplier, multiplier == 1), -1, 1));
    
    // double corrected_heading = gyro.getAngle();
    double corrected_heading = gyro.getAngle();
    // System.out.println("heading" + corrected_heading);
    boolean negative = corrected_heading < 0;
    corrected_heading = Math.abs(corrected_heading);
    double reference = corrected_heading % 360;
    if (negative) {
      reference = -reference;
    }

    double botHeading = reference * Math.PI / 180.0;
    if (RobotContainer.oi.driver.getBButton()) {
      gyro_offset = botHeading;
    }
    botHeading -= gyro_offset;
    // m_odometry.update(new Rotation2d(corrected_heading), new MecanumDriveWheelSpeeds(leftFront.getEncoder().getVelocity(), rightFront.getEncoder().getVelocity(), leftBack.getEncoder().getVelocity(), rightBack.getEncoder().getVelocity()));

    double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
    double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
    double frontLeftPower = (rotY + rotX + rx) / denominator;
    double backLeftPower = (rotY - rotX + rx) / denominator;
    double frontRightPower = (rotY - rotX - rx) / denominator;
    double backRightPower = (rotY + rotX - rx) / denominator;

    // leftFront.set(clamp(curveInput(PID.motor_1 * frontLeftPower * multiplier, multiplier == 1, curve_b), -1, 1));
    // leftBack.set(clamp(curveInput(PID.motor_4 * backLeftPower * multiplier, multiplier == 1, curve_b), -1, 1));
    // rightFront.set(clamp(curveInput(PID.motor_2 * frontRightPower * multiplier, multiplier == 1, curve_b), -1, 1));
    // rightBack.set(clamp(curveInput(PID.motor_4 * backRightPower * multiplier, multiplier == 1, curve_b), -1, 1));
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
   * Call to arcade drive the robot with a power and rotation
   * @param power drive speed
   * @param z rotation speed
   */
  public void drive(double power, double z){
    
  }

  /**
   * sets the max output of the DifferentialDrive object (scaling factor when driving robot)
   * @param maxOutput max output to set
   */
  public void setMaxOutput(double maxOutput){
    // m_drivetrain.setMaxOutput(maxOutput);
  }


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
    leftBack.getEncoder().setPosition(0);
    rightFront.getEncoder().setPosition(0);
    rightBack.getEncoder().setPosition(0);
    leftFront.getEncoder().setPosition(0);
  }

  // only for front back
  public double encoderDistance() {
    return (leftFront.getEncoder().getPosition() + -rightFront.getEncoder().getPosition() + -rightBack.getEncoder().getPosition() + leftBack.getEncoder().getPosition()) / 4;
  }

  public void clearPowers() {
    leftFront.set(0);
    rightFront.set(0);
    rightBack.set(0);
    leftBack.set(0);
  }

  // distance in feet
  public void holoDrive(double power) {
    rightFront.setInverted(true);
    rightBack.setInverted(true);
    
    //   var wheelPositions = new MecanumDriveWheelPositions(
    //   leftFront.getEncoder().getPosition(), rightFront.getEncoder().getPosition(),
    //   leftBack.getEncoder().getPosition(), rightBack.getEncoder().getPosition());

    // // Get the rotation of the robot from the gyro.
    // var gyroAngle = gyro.getAngle();
    // System.out.println(gyroAngle);

    // Update the pose
    double y = -power; // Remember, this is reversed!
    double x = 0; // Counteract imperfect strafing
    double rx = 0;
    
    // double corrected_heading = gyro.getAngle();
    double corrected_heading = gyro.getAngle();
    // System.out.println("heading" + corrected_heading);
    boolean negative = corrected_heading < 0;
    corrected_heading = Math.abs(corrected_heading);
    double reference = corrected_heading % 360;
    if (negative) {
      reference = -reference;
    }

    double multiplier = 0.9;

    double botHeading = 0;
    // m_odometry.update(new Rotation2d(corrected_heading), new MecanumDriveWheelSpeeds(leftFront.getEncoder().getVelocity(), rightFront.getEncoder().getVelocity(), leftBack.getEncoder().getVelocity(), rightBack.getEncoder().getVelocity()));

    double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
    double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
    double frontLeftPower = (rotY + rotX + rx) / denominator;
    double backLeftPower = (rotY - rotX + rx) / denominator;
    double frontRightPower = (rotY - rotX - rx) / denominator;
    double backRightPower = (rotY + rotX - rx) / denominator;

    leftFront.set(clamp(curveInput(PID.motor_1 * frontLeftPower * multiplier, multiplier == 1, curve_b), -1, 1));
    leftBack.set(clamp(curveInput(PID.motor_4 * backLeftPower * multiplier, multiplier == 1, curve_b), -1, 1));
    rightFront.set(clamp(curveInput(PID.motor_2 * frontRightPower * multiplier, multiplier == 1, curve_b), -1, 1));
    rightBack.set(clamp(curveInput(PID.motor_4 * backRightPower * multiplier, multiplier == 1, curve_b), -1, 1));
  }


  /**
   * Add a trajectory to the field object that will be displayed on the smart dashboard
   * @param trajectory Trajectory to add
   */
  public void putTrajectory(Trajectory trajectory){
    field.getObject("traj").setTrajectory(trajectory);
  }
}