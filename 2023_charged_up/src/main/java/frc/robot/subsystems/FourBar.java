package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class FourBar extends SubsystemBase {
    CANSparkMax fourbarNeo = new CANSparkMax(Constants.fourbar_neo, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    RelativeEncoder fourbarEncoder;
    SparkMaxPIDController fourbarPID;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc, minVel, allowedErr;
    public double previousEncoder = 0;
    public boolean extended = false;

    double targetExtend = 2;
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
        fourbarNeo.setIdleMode(IdleMode.kBrake);
        fourbarEncoder = fourbarNeo.getEncoder();
        fourbarPID = fourbarNeo.getPIDController();
        // 0 is ~upright
        fourbarEncoder.setPosition(0);
        // conversion is in degrees
        // 45:1 -> 40:20
        // 90:1
        // 360/90 -> 1 rotation = 4 degrees
        fourbarEncoder.setPositionConversionFactor(4);
        fourbarPID.setFeedbackDevice(fourbarEncoder);
        kP = 0.004; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 0.4; 
        kMinOutput = -0.4;

        maxVel = 10 ; // rpm
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

    public void run() {
        targetExtend -= 10;
    }

    @Override
    public void periodic() {
        if (RobotContainer.oi.driver.getPOV() == 270 && targetExtend > -3) {
            targetExtend -= 0.5;
        } else if (RobotContainer.oi.driver.getPOV() == 90 && targetExtend < 74) {
            targetExtend += 0.5;
        }
        SmartDashboard.putNumber("pov", RobotContainer.oi.driver.getPOV());
        SmartDashboard.putNumber("neoamp", fourbarNeo.getOutputCurrent());
        SmartDashboard.putNumber("neoencoder", fourbarEncoder.getPosition());
        SmartDashboard.putNumber("target",targetExtend);
        if (fourbarNeo.getOutputCurrent() < 4) {
            fourbarPID.setReference(targetExtend, CANSparkMax.ControlType.kPosition);
            fourbarNeo.setIdleMode(IdleMode.kBrake);
        } else {
            fourbarNeo.setIdleMode(IdleMode.kCoast);
            // fourbarNeo.set(0);
        }
        
    }
}
