package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc, minVel, allowedErr;
    // Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    // positive power goes up
    CANSparkMax rotateNeo = new CANSparkMax(Constants.intake_rotate, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private RelativeEncoder rotateEncoder = rotateNeo.getEncoder();
    CANSparkMax intakeNeo = new CANSparkMax(Constants.intake_intake, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    private SparkMaxPIDController rotatePID = rotateNeo.getPIDController();

    public boolean scoring = false;
    public double power = 0.5;

    DoubleSolenoid climbSolenoid = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 15, 13);
    boolean extended = false;
    // boolean compressor_status = false;
    // true is down
    boolean flipped = false;
    double initial_position = 0;
    double angle = 80;

    // cone scoring for high goal when pushed against 2x4s, 15 degrees 0.5 power

    public Intake() {
        rotateEncoder.setPosition(0);
        rotateEncoder.setInverted(true);
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
        kP = 0.005; 
        kI = 0.000001;
        kD = 0.000001; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 0.35; 
        kMinOutput = -0.35;

        maxVel = 50; // rpm
        maxAcc = 10;

        int smartMotionSlot = 0;
        rotatePID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        rotatePID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        rotatePID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        rotatePID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);


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

    public void run_intake_in() {
       intakeNeo.set(0.4); 
    }

    public void run_intake_out() {
       intakeNeo.set(-1); 
    }

    public void flip_intake() {
        flipped = !flipped;
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
         // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        angle = SmartDashboard.getNumber("angle ", angle);
        power = SmartDashboard.getNumber("power ", power);
        
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        // if((p != kP)) { rotatePID.setP(p); kP = p; }
        // if((i != kI)) { rotatePID.setI(i); kI = i; }
        // if((d != kD)) { rotatePID.setD(d); kD = d; }
        // if((iz != kIz)) { rotatePID.setIZone(iz); kIz = iz; }
        // if((ff != kFF)) { rotatePID.setFF(ff); kFF = ff; }
        // if((max != kMaxOutput) || (min != kMin6Output)) { 
        //     rotatePID.setOutputRange(min, max); 
        // kMinOutput = min; kMaxOutput = max; 
        // }
        // rotatePID.setFeedbackDevice(rotateEncoder);
        // System.out.println(flipped);
        if (flipped) {
            rotatePID.setReference(angle, CANSparkMax.ControlType.kPosition);
        } else if (!flipped) {
            rotatePID.setReference(0, CANSparkMax.ControlType.kPosition);
        } else {
            rotateNeo.set(0);
        }
        if (scoring) {
            intakeNeo.set(power);
        }  else {
            intakeNeo.set(0);
        }
    
        // System.out.println(rotateNeo.getEncoder().getPosition());
        // intakeNeo.set(0);
    }

    public void toggle() {
        climbSolenoid.toggle();
    }
}
