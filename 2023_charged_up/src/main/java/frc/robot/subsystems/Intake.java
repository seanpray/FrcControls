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

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc, minVel, allowedErr;
    Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    // positive power goes up
    CANSparkMax rotateNeo = new CANSparkMax(Constants.intake_rotate, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private RelativeEncoder rotateEncoder = rotateNeo.getEncoder();
    CANSparkMax intakeNeo = new CANSparkMax(Constants.intake_intake, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    public double level = -1;
    public SparkMaxPIDController rotatePID = rotateNeo.getPIDController();

    public double power = 0.5;

    public boolean up = false;

    DoubleSolenoid climbSolenoid = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 1, 2);
    boolean extended = false;
    // boolean compressor_status = false;
    // true is down
    double initial_position = 0;
    public double angle = -15;
    public double previousEncoder = 0;
    public double stallEpsilon = 3;

    DigitalInput intakeSwitch = new DigitalInput(0);

     // true if "stalled"
     public boolean motorEndstop(double motorPower, double encoderValue) {
        return Math.abs(previousEncoder - encoderValue) > stallEpsilon && Math.abs(motorPower) > 0.1;
    }

    // cone scoring for high goal when pushed against 2x4s, 15 degrees 0.5 power


    public void down() {
        angle -= 0.05;
    }

    public Intake() {
         rotateEncoder.setPosition(0);
        // if (intakeSwitch.get()) {
        //     rotateEncoder.setPosition(-90);
        //     level = 0;
        // }
        rotateNeo.setIdleMode(IdleMode.kBrake);
       
        initial_position = rotateEncoder.getPosition();
        rotateNeo.restoreFactoryDefaults();
        intakeNeo.restoreFactoryDefaults(); 
        climbSolenoid.set(DoubleSolenoid.Value.kForward);
        // in degrees, 1 neo rotation is 18 degrees of intake rotation
        rotateEncoder.setPositionConversionFactor(18);
        // rotateNeo.setInverted(true);
        compressor.enableDigital();
        // compressor_status = true;

        // SmartDashboard.putNumber("angle ", angle);
        // SmartDashboard.putNumber("power ", power);

        // PID coefficients
        kP = 0.008; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 0.4; 
        
        kMinOutput = -0.4;
        // SmartDashboard.putNumber("intake kP", kP);
        // SmartDashboard.putNumber("intake kD", kD);
        // SmartDashboard.putNumber("intake kD", kD);

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

    public void runIntakeIn(double power) {
       intakeNeo.set(power); 
    }

    public void runIntakeOut(double power) {
       intakeNeo.set(-power); 
    }

    public void set_angle(double newangle) {
        double a = newangle > 80 ? 80 : newangle < 0 ? 0 : newangle;
        level = -1;
        angle = -a;
        // rotatePID.setReference(-a, CANSparkMax.ControlType.kPosition);
    }

    public void setIntakeLevel() {
        level++;
        if (level > 1) {
            level = 0;
        }
    }

    public void raise() {
        angle += 0.5;
    }

    @Override
    public void periodic() {
        rotatePID.setReference(angle, CANSparkMax.ControlType.kPosition);
        if (level == 0) {
            angle = -15;
        } else if (level == 1) {
            angle = -80;
        }
        SmartDashboard.putNumber("intake rotation", getIntakePos());
    }

    public double getIntakePos(){
        return rotateEncoder.getPosition();
    }

    public void toggle() {
        climbSolenoid.toggle();
    }
}
