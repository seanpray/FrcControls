package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.RobotContainer;

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

    public boolean toggle_up = false;

    public double elevatorTarget = 0;

    // positive power goes 
    CANSparkMax spoolNeo = new CANSparkMax(Constants.elevator_neo, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    RelativeEncoder spoolEncoder;
    
    private SparkMaxPIDController spoolPID;

    public ElevatorLevel targetState = ElevatorLevel.GrabCone;


    public void set_target(ElevatorLevel target) {
        targetState = target;
    }

    public void toggle_elevator() {
        toggle_up = !toggle_up;
    }

    public Elevator() {
        spoolEncoder = spoolNeo.getEncoder();
        spoolEncoder.setPosition(0);
        spoolPID = spoolNeo.getPIDController();
        spoolNeo.restoreFactoryDefaults();
        // 4PI per a rotation, 12:96 pinion: gear, 96/12 -> 96/12 * 4 * PI
        spoolNeo.getEncoder().setPositionConversionFactor(0.125 * 4 * Math.PI);
        spoolPID.setFeedbackDevice(spoolEncoder);

        // PID coefficients
        kP = 0.006; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 0.8; 
        kMinOutput = -0.8;

        maxVel = 1; // rpm
        maxAcc = 1;

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
        if (RobotContainer.oi.driver.getPOV() == 0) {
            if (elevatorTarget > -52) {
                elevatorTarget -= 0.8;
            }
        } else if (Math.abs(RobotContainer.oi.driver.getPOV() - 180) < 10) {
            if (elevatorTarget < 40) {
                elevatorTarget += 0.8;
            }
        }
        // System.out.println(elevatorTarget);
         // read PID coefficients from SmartDashboard
        // double p = SmartDashboard.getNumber("Elevator P Gain", 0);
        // double i = SmartDashboard.getNumber("Elevator I Gain", 0);
        // double d = SmartDashboard.getNumber("Elevator D Gain", 0);
        // double iz = SmartDashboard.getNumber("Elevator I Zone", 0);
        // double ff = SmartDashboard.getNumber("Elevator Feed Forward", 0);
        // double max = SmartDashboard.getNumber("Elevator Max Output", 0);
        // double min = SmartDashboard.getNumber("Elevator Min Output", 0);
        // if (elevatorTopLimit.get()) {
        //     // active
        // }
        spoolPID.setReference(elevatorTarget, CANSparkMax.ControlType.kPosition);
        // if (toggle_up) {
        //     spoolPID.setReference(elevatorTarget, CANSparkMax.ControlType.kPosition);
        // } else {
        //     // spoolNeo.set(0);
        // }
        
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
        // switch (targetState) {
        //     case GrabCone:
        //         resetOutputRange();
        //         spoolPID.setReference(0, CANSparkMax.ControlType.kPosition);
        //         break;
        //     case Low:
        //         resetOutputRange();
        //         spoolPID.setReference(4, CANSparkMax.ControlType.kPosition);
        //         break;
        //     case Medium:
        //         resetOutputRange();
        //         spoolPID.setReference(5, CANSparkMax.ControlType.kPosition);
        //         break;
        //     default:
        //         spoolPID.setOutputRange(0, 0);
        //         break;
        // }
        // System.out.println(spoolEncoder.getPosition());
        // if (targetState == ElevatorLevel.Medium && elevatorTopLimit.get()) {
            // spoolEncoder.setPosition(min)
        // }
    }

}
