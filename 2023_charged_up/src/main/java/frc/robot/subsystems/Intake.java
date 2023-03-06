package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    public static final int neo_rotations = 1350;
    public static final int intake_rotationes = 19;
    Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    CANSparkMax rotateNeo = new CANSparkMax(Constants.intake_rotate, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax intakeNeo = new CANSparkMax(Constants.intake_intake, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    DoubleSolenoid climbSolenoid = new DoubleSolenoid(9, PneumaticsModuleType.REVPH, 0,1);
    boolean extended = false;
    boolean compressor_status = false;
    boolean flipped = false;
    double initial_position = 0;

    public Intake() {
        initial_position = rotateNeo.getEncoder().getPosition();
        rotateNeo.restoreFactoryDefaults();
        climbSolenoid.set(DoubleSolenoid.Value.kForward);
        compressor.enableDigital();
        compressor_status = true;
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

    public void toggleCompressor() {
        if (compressor_status) {
            compressor.disable();
        } else {
            compressor.enableDigital();
        }
        compressor_status = !compressor_status;
    }

    @Override
    public void periodic() {
        double position = rotateNeo.getEncoder().getPosition();
        // position -= initial_position;
        // if (flipped && position > -18) {
        //     rotateNeo.set(-0.5);
        // } else if (!flipped && position < -3) {
        //     rotateNeo.set(0.5);
        // } else {
        //     rotateNeo.set(0);
        // }
        rotateNeo.set(0);
        intakeNeo.set(0);
    }

    public void toggle() {
        if (extended) {
            climbSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            climbSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        extended = !extended;
        // climbSolenoid.toggle();
    }
}
