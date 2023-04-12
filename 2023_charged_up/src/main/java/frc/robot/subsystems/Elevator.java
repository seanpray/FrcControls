package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class Elevator extends SubsystemBase {
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc, minVel, allowedErr;
    DigitalInput elevatorTopLimit = new DigitalInput(4);

    public boolean toggle_up = false;

    public int elevatorLevel = 0;
    // 0 is unmanaged
    // 1 is pickup/score
    // 2 is store

    public double elevatorTarget = 0;
    DigitalInput elevatorSwitchTop = new DigitalInput(1);
    DigitalInput elevatorSwitchBottom = new DigitalInput(2);

    // positive power goes
    CANSparkMax spoolNeo = new CANSparkMax(Constants.elevator_neo,
            com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    RelativeEncoder spoolEncoder;

    private SparkMaxPIDController spoolPID;

    public void toggle_elevator() {
        toggle_up = !toggle_up;
    }

    public boolean home = false;

    public void homeElevator() {
        home = true;
    }

    public Elevator() {
        spoolEncoder = spoolNeo.getEncoder();
        spoolEncoder.setPosition(0);
        spoolNeo.restoreFactoryDefaults();
        spoolNeo.setInverted(true);
        spoolPID = spoolNeo.getPIDController();
        
        // 4PI per a rotation, 12:96 pinion: gear, 96/12 -> 96/12 * 4 * PI
        spoolNeo.getEncoder().setPositionConversionFactor(0.125 * 4 * Math.PI);
        spoolPID.setFeedbackDevice(spoolEncoder);
        // spoolEncoder.setInverted(true);

        // PID coefficients
        kP = 0.04;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 0.5;
        kMinOutput = -0.5;

        maxVel = 2; // rpm
        maxAcc = 3;

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

    public void resetEncoder() {
        spoolEncoder.setPosition(0);
    }

    public void pullUp(double amount) {
        elevatorTarget -= amount;
    }

    public void elevatorPreset() {
        changeElevatorTarget = true;
        elevatorLevel++;
        if (elevatorLevel > 2) {
            elevatorLevel = 1;
        }
    }

    boolean changeElevatorTarget = false;

    @Override
    public void periodic() {
        if (changeElevatorTarget) {
            RobotContainer.intake.set_angle(40);
        }
        switch (elevatorLevel) {
            case 2:
                elevatorTarget = -8;
                break;
            case 1:
                elevatorTarget = -26;
                break;
        }
        if (elevatorSwitchBottom.get() && elevatorSwitchTop.get()) {
            home = false;
            // spoolEncoder.setPosition(0);
        }
        changeElevatorTarget = false;
        // SmartDashboard.putBoolean("eswitcht", elevatorSwitchTop.get());
        //  SmartDashboard.putBoolean("eswitchB", elevatorSwitchBottom.get());
        //  SmartDashboard.putNumber("eencoder",spoolEncoder.getPosition());
        //  SmartDashboard.putNumber("elec", spoolNeo.getOutputCurrent());
        if (RobotContainer.oi.driver.getPOV() == 0 || RobotContainer.oi.operator.getPOV() == 0) {
            elevatorLevel = 0;
            if (!(elevatorSwitchBottom.get() && elevatorSwitchTop.get()) && spoolNeo.getOutputCurrent() < 85) {
                elevatorTarget -= 1;
            }
        } else if (RobotContainer.oi.driver.getPOV() == 180 || RobotContainer.oi.operator.getPOV() == 180) {
            elevatorLevel = 0;
            if (elevatorTarget < 50) {
                elevatorTarget += 1;
            }
        }

        spoolPID.setReference(elevatorTarget, CANSparkMax.ControlType.kPosition);
    }

}
