package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake extends SubsystemBase {

    public boolean auton = false;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc, minVel, allowedErr;
    Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    // positive power goes up
    CANSparkMax rotateNeo = new CANSparkMax(Constants.intake_rotate, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private RelativeEncoder rotateEncoder = rotateNeo.getEncoder();
    CANSparkMax intakeNeo = new CANSparkMax(Constants.intake_intake, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    double level = -1;
    private SparkMaxPIDController rotatePID = rotateNeo.getPIDController();

    public double power = 0.5;

    public boolean up = false;

    DoubleSolenoid climbSolenoid = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 1, 2);
    boolean extended = false;
    // boolean compressor_status = false;
    // true is down
    double initial_position = 0;
    double angle = -15;
    public double previousEncoder = 0;
    public double stallEpsilon = 3;

    public void auton(boolean state) {
        auton = state;
        if (!auton) {
            level = 0;
        } else {
            level = -1;
        }
    }

    DigitalInput intakeSwitch = new DigitalInput(0);

     // true if "stalled"
     public boolean motorEndstop(double motorPower, double encoderValue) {
        return Math.abs(previousEncoder - encoderValue) > stallEpsilon && Math.abs(motorPower) > 0.1;
    }

    // cone scoring for high goal when pushed against 2x4s, 15 degrees 0.5 power


    public void down() {
        if (!intakeSwitch.get()) {
            angle -= 0.05;
        }
    }

    public Intake() {
        rotateNeo.setIdleMode(IdleMode.kBrake);
        rotateEncoder.setPosition(0);
        initial_position = rotateEncoder.getPosition();
        rotateNeo.restoreFactoryDefaults();
        intakeNeo.restoreFactoryDefaults();
        climbSolenoid.set(DoubleSolenoid.Value.kForward);
        // in degrees, 1 neo rotation is 18 degrees of intake rotation
        rotateNeo.getEncoder().setPositionConversionFactor(18);
        compressor.enableDigital();
        // compressor_status = true;

        SmartDashboard.putNumber("angle ", angle);
        SmartDashboard.putNumber("power ", power);

        // PID coefficients
        kP = 0.02; 
        kI = 0.000035;
        kD = 0.00033; 
        kIz = 0; 
        kFF = 0.00005; 
        kMaxOutput = 0.35; 
        kMinOutput = -0.35;
        SmartDashboard.putNumber("intake kP", kP);
        SmartDashboard.putNumber("intake kD", kD);
        SmartDashboard.putNumber("intake kD", kD);

        maxVel = 3; // rpm
        maxAcc = 2;

        int smartMotionSlot = 0;
        rotatePID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        rotatePID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        rotatePID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        rotatePID.setSmartMotionAllowedClosedLoopError(4, smartMotionSlot);


        rotatePID.setP(kP);
        rotatePID.setI(kI);
        rotatePID.setD(kD);
        rotatePID.setIZone(kIz);
        rotatePID.setFF(kFF);
        rotatePID.setOutputRange(kMinOutput, kMaxOutput);

    }

    public void resetEncoder() {
        // rotateEncoder.setPosition(0);
    }

    public void run_intake_in(double power) {
       intakeNeo.set(-power); 
    }

    public void run_intake_out(double power) {
       intakeNeo.set(power); 
    }

    public void set_angle(double newangle) {
        double a = newangle > 80 ? 80 : newangle < 0 ? 0 : newangle;
        angle = -a;
        // rotatePID.setReference(-a, CANSparkMax.ControlType.kPosition);
    }

    public void setIntakeLevel() {
        level++;
        if (level > 1) {
            level = 0;
        }
    }

    @Override
    public void periodic() {
        if (RobotContainer.oi.driver.getPOV() == 90) {
            if (intakeSwitch.get()) {
                rotateNeo.getEncoder().setPosition(-80);
            } else {
                angle -= 0.2;
            }
        }
        power = SmartDashboard.getNumber("power ", power);

        // double outtake = 0;
        if (level >= 0) {
            auton = false;
        }
        if (level == 0) {
            angle = -15;
        } else if (level == 1) {
            angle = -80;
        }
        if (!auton) {
            double outtake = RobotContainer.oi.driver.getLeftTriggerAxis();
            double intake = outtake > 0.1 ? outtake < 0.1 ? -0.1 : -outtake * 0.7 : RobotContainer.oi.driver.getRightTriggerAxis() > 0.1 ? 0.53 : 0;
            if (Math.abs(intake) > 0.1) {
                intakeNeo.set(intake);
            } else {
                intakeNeo.set(-0.08);
            }
        }
        // System.out.println(auton);
        // System.out.println(angle);
        rotatePID.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public void toggle() {
        climbSolenoid.toggle();
    }
}
