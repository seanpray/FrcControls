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

public class Intake extends SubsystemBase {
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc, minVel, allowedErr;
    // Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    // positive power goes up
    CANSparkMax rotateNeo = new CANSparkMax(Constants.intake_rotate, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private RelativeEncoder rotateEncoder = rotateNeo.getEncoder();
    CANSparkMax intakeNeo = new CANSparkMax(Constants.intake_intake, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    double level = 0;
    private SparkMaxPIDController rotatePID = rotateNeo.getPIDController();

    public boolean scoring = false;
    public double power = 0.5;

    Timer time = new Timer();

    public boolean up = false;

    DoubleSolenoid climbSolenoid = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 1, 2);
    boolean extended = false;
    // boolean compressor_status = false;
    // true is down
    boolean flipped = false;
    double initial_position = 0;
    double angle = -80;
    public double previousEncoder = 0;
    public double stallEpsilon = 3;

     // true if "stalled"
     public boolean motorEndstop(double motorPower, double encoderValue) {
        return Math.abs(previousEncoder - encoderValue) > stallEpsilon && Math.abs(motorPower) > 0.1;
    }

    // cone scoring for high goal when pushed against 2x4s, 15 degrees 0.5 power

    public Intake() {
        rotateNeo.setIdleMode(IdleMode.kBrake);
        time.start();
        rotateEncoder.setPosition(0);
        initial_position = rotateEncoder.getPosition();
        rotateNeo.restoreFactoryDefaults();
        intakeNeo.restoreFactoryDefaults();
        climbSolenoid.set(DoubleSolenoid.Value.kForward);
        // in degrees, 1 neo rotation is 18 degrees of intake rotation
        rotateNeo.getEncoder().setPositionConversionFactor(18);
        // compressor.enableDigital();
        // compressor_status = true;

        SmartDashboard.putNumber("angle ", angle);
        SmartDashboard.putNumber("power ", power);

        // PID coefficients
        kP = 0.02; 
        kI = 0.000035;
        kD = 0.00033; 
        kIz = 0; 
        kFF = 0.00005; 
        kMaxOutput = 0.4; 
        kMinOutput = -0.4;
        SmartDashboard.putNumber("intake kP", kP);
        SmartDashboard.putNumber("intake kD", kD);
        SmartDashboard.putNumber("intake kD", kD);

        maxVel = 4; // rpm
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

    public void score() {
        scoring = !scoring;
    }

    public void resetEncoder() {
        rotateEncoder.setPosition(0);
    }

    public void run_intake_in(double power) {
       intakeNeo.set(power); 
    }

    public void run_intake_out(double power) {
       intakeNeo.set(-power); 
    }

    public void set_angle(double newangle) {
        double a = newangle > 80 ? 80 : newangle < 0 ? 0 : newangle;
        rotatePID.setReference(-a, CANSparkMax.ControlType.kPosition);
    }

    public void flip_intake() {
        level++;
        // flipped = !flipped;
        if (level > 2) {
            level = 0;
        }
    }

    // public void toggleCompressor() {
    //     if (compressor_status) {
    //         compressor.disable();
    //     } else {
    //         compressor.enableDigital();
    //     }
    //     compressor_status = !compressor_status;
    // }

    @Override
    public void periodic() {
        power = SmartDashboard.getNumber("power ", power);
        // double outtake = RobotContainer.oi.driver.getLeftTriggerAxis();
        double outtake = 0;
        double intake = outtake > 0.05 ? outtake < 0.1 ? -0.1 : -outtake : RobotContainer.oi.driver.getRightTriggerAxis() > 0.05 ? 0.55 : 0;
        if (Math.abs(intake) > 0.05) {
            intakeNeo.set(intake);
        } else {
            intakeNeo.set(0);
        }
        if (level == 0) {
            angle = -85;
        } else if (level == 1) {
            angle = -50;
        } else {
            angle = -15;
        }
        rotatePID.setReference(angle, CANSparkMax.ControlType.kPosition);

         // read PID coefficients from SmartDashboard
        // angle = SmartDashboard.getNumber("angle ", angle);
       
        
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        // if((p != kP)) { rotatePID.setP(p); kP = p; }
        // if((i != kI)) { rotatePID.setI(i); kI = i; }
        // if((d != kD)) { rotatePID.setD(d); kD = d; }
        // if((iz != kIz)) { rotatePID.setIZone(iz); kIz = iz; }
        // if((ff != kFF)) { rotatePID.setFF(ff); kFF = ff; }
        // if((max != kMaxOutput) || (min != kMinOutput)) { 
        //     rotatePID.setOutputRange(min, max); 
        // kMinOutput = min; kMaxOutput = max; 
        // }
        // rotatePID.setFeedbackDevice(rotateEncoder);
        // System.out.println(flipped);
        // if (flipped) {
        //     rotatePID.setReference(angle, CANSparkMax.ControlType.kPosition);
        // } else if (!flipped) {
        //     rotatePID.setReference(-30, CANSparkMax.ControlType.kPosition);
        // } else {
        //     rotateNeo.set(0);
        // }
        // if (motorEndstop(rotateNeo.getAppliedOutput(), rotateEncoder.getPosition()) && rotateEncoder.getPosition() > -8) {
        //     rotateEncoder.setPosition(0);
        // }
        // if (scoring) {
        //     intakeNeo.set(power);
        // }  else {
        //     intakeNeo.set(0);
        // }
        // System.out.println(rotateNeo.getEncoder().getPosition());
        // intakeNeo.set(0);
    }

    public void toggle() {
        climbSolenoid.toggle();
    }
}
