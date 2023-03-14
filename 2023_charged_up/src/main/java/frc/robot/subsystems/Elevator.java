package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

enum ElevatorLevel {
    // the level in which we can get the cone from the intake into the four bar
    GrabCone,
    // low goal, basically just forward without lifting up much
    Low,
    // medium goal
    Medium,
    // turn off PID
    Idle,
}

public class Elevator extends SubsystemBase {
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc, minVel, allowedErr;
    DigitalInput elevatorTopLimit = new DigitalInput(4);

    // positive power goes 
    CANSparkMax spoolNeo = new CANSparkMax(Constants.intake_rotate, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    RelativeEncoder spoolEncoder = spoolNeo.getEncoder();
    
    private SparkMaxPIDController spoolPID = spoolNeo.getPIDController();

    public ElevatorLevel targetState = ElevatorLevel.GrabCone;

    public void set_target(ElevatorLevel target) {
        targetState = target;
    }

    public Elevator() {
        spoolEncoder.setPosition(0);
        spoolNeo.restoreFactoryDefaults();
        // 4PI per a rotation, 12:96 pinion: gear, 96/12 -> 96/12 * 4 * PI
        spoolNeo.getEncoder().setPositionConversionFactor(8 * 4 * Math.PI);
        spoolPID.setFeedbackDevice(spoolEncoder);

        // PID coefficients
        kP = 0.001; 
        kI = 0.0001;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 0.1; 
        kMinOutput = -0.1;

        maxVel = 50; // rpm
        maxAcc = 10;

        int smartMotionSlot = 0;
        spoolPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        spoolPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        spoolPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        spoolPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);


        spoolPID.setP(kP);
        spoolPID.setI(kI);
        spoolPID.setD(kD);
        spoolPID.setIZone(kIz);
        spoolPID.setFF(kFF);
        spoolPID.setOutputRange(kMinOutput, kMaxOutput);

    }

    public void resetOutputRange() {
        spoolPID.setOutputRange(kMinOutput, kMaxOutput); 
    }
    @Override
    public void periodic() {
        
         // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("Elevator P Gain", 0);
        double i = SmartDashboard.getNumber("Elevator I Gain", 0);
        double d = SmartDashboard.getNumber("Elevator D Gain", 0);
        double iz = SmartDashboard.getNumber("Elevator I Zone", 0);
        double ff = SmartDashboard.getNumber("Elevator Feed Forward", 0);
        double max = SmartDashboard.getNumber("Elevator Max Output", 0);
        double min = SmartDashboard.getNumber("Elevator Min Output", 0);
        if (elevatorTopLimit.get()) {
            // active
        }
        
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        // if((p != kP)) { spoolPID.setP(p); kP = p; }
        // if((i != kI)) { spoolPID.setI(i); kI = i; }
        // if((d != kD)) { spoolPID.setD(d); kD = d; }
        // if((iz != kIz)) { spoolPID.setIZone(iz); kIz = iz; }
        // if((ff != kFF)) { spoolPID.setFF(ff); kFF = ff; }
        // if((max != kMaxOutput) || (min != kMinOutput)) { 
        //     spoolPID.setOutputRange(min, max); 
        // kMinOutput = min; kMaxOutput = max; 
        // }
        // if (flipped) {
        //     spoolPID.setReference(-40, CANSparkMax.ControlType.kPosition);
        // } else if (!flipped) {
        //     spoolPID.setReference(0, CANSparkMax.ControlType.kPosition);
        // } else {
        //     rotateNeo.set(0);
        // }
        // reference input is inches, I think
        switch (targetState) {
            case GrabCone:
                resetOutputRange();
                spoolPID.setReference(0, CANSparkMax.ControlType.kPosition);
                break;
            case Low:
                resetOutputRange();
                spoolPID.setReference(4, CANSparkMax.ControlType.kPosition);
                break;
            case Medium:
                resetOutputRange();
                spoolPID.setReference(5, CANSparkMax.ControlType.kPosition);
                break;
            default:
                spoolPID.setOutputRange(0, 0);
                break;
        }
    }

}
