package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class FourBar extends SubsystemBase {
    CANSparkMax neo550 = new CANSparkMax(Constants.fourbar_neo, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    RelativeEncoder fourbarEncoder;
    SparkMaxPIDController fourbarPID;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc, minVel, allowedErr;
    public double previousEncoder = 0;
    public boolean extended = false;

    double targetExtend = 40;
    // within 4 degrees
    public double stallEpsilon = 1;

    // true if "stalled"
    public boolean motorEndstop(double motorPower, double encoderValue) {
        return Math.abs(previousEncoder - encoderValue) < stallEpsilon && Math.abs(motorPower) > 0.1;
    }

    public void toggleExtend() {
        extended = !extended;
    }

    public void resetEncoder() {
        fourbarEncoder.setPosition(0);
    }

    public FourBar() {
        fourbarEncoder = neo550.getEncoder();
        fourbarPID = neo550.getPIDController();
        // 0 is ~upright
        fourbarEncoder.setPosition(0);
        // conversion is in degrees
        // 45:1 -> 40:20
        // 90:1
        // 360/90 -> 1 rotation = 4 degrees
        fourbarEncoder.setPositionConversionFactor(4);
        fourbarPID.setFeedbackDevice(fourbarEncoder);
        kP = 0.004; 
        kI = 0.000002;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 0.8; 
        kMinOutput = -0.8;

        maxVel = 30; // rpm
        maxAcc = 5;

        int smartMotionSlot = 0;
        fourbarPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        fourbarPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        fourbarPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        fourbarPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);


        fourbarPID.setP(kP);
        fourbarPID.setI(kI);
        fourbarPID.setD(kD);
        fourbarPID.setIZone(kIz);
        fourbarPID.setFF(kFF);
        fourbarPID.setOutputRange(kMinOutput, kMaxOutput);
    }

    @Override
    public void periodic() {
        // System.out.println(RobotContainer.oi.driver.getPOV());
        if (Math.abs(RobotContainer.oi.driver.getPOV() - 90) < 10) {
            if (targetExtend > 2) {
                targetExtend -= 0.5;
            }
        } else if (Math.abs(RobotContainer.oi.driver.getPOV() - 270) < 10) {
            if (targetExtend < 100) {
                targetExtend += 0.5;
            }
        }
        // TODO check if it's just get() or getAppliedOutput()
        // if (motorEndstop(neo550.getAppliedOutput(), fourbarEncoder.getPosition()) && fourbarEncoder.getPosition() < 45) {
        //     fourbarEncoder.setPosition(0);
        // }
        
        if (extended) {
            fourbarPID.setReference(targetExtend, CANSparkMax.ControlType.kPosition);
            // fourbarPID.setReference(50, CANSparkMax.ControlType.kPosition);
        } else { 
            fourbarPID.setReference(0, CANSparkMax.ControlType.kPosition);
        }
    }
}
