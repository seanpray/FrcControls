package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    DigitalInput elevatorSwitchTop = new DigitalInput(1);
    DigitalInput elevatorSwitchBottom = new DigitalInput(2);

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
        kMaxOutput = 0.9; 
        kMinOutput = -0.9;

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

    public void resetEncoder() {
        spoolEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        if (elevatorSwitchBottom.get() && elevatorSwitchTop.get()) {
            spoolEncoder.setPosition(0);
        }
        SmartDashboard.putBoolean("eswitcht", elevatorSwitchTop.get());
         SmartDashboard.putBoolean("eswitchB", elevatorSwitchBottom.get());
         SmartDashboard.putNumber("elec", spoolNeo.getOutputCurrent());
        if (RobotContainer.oi.driver.getPOV() == 0) {
            if (!elevatorSwitchBottom.get() || !elevatorSwitchTop.get() && spoolNeo.getOutputCurrent() < 30) {
                elevatorTarget -= 1.2;
            }
        } else if (RobotContainer.oi.driver.getPOV() == 180) {
            if (elevatorTarget < 70) {
                elevatorTarget += 1.2;
            }
        }

        spoolPID.setReference(elevatorTarget, CANSparkMax.ControlType.kPosition);
    }

}
