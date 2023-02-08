package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;

public class Telemetry {
    public Telemetry() {
        
    }

    public void update() {
    }
    public void pid() {
        ShuffleboardTab tab = Shuffleboard.getTab("Motor 1");
        NetworkTableEntry shooterEnable = tab.add("PID 1 Enable", true).getEntry();

        // Command Example assumed to be in a PIDSubsystem
        new NetworkButton(shooterEnable).onTrue(new InstantCommand(m_shooter::enable));

        // Timed Robot Example
        if (shooterEnable.getBoolean()) {
        // Calculates the output of the PID algorithm based on the sensor reading
        // and sends it to a motor
            motor.set(pid.calculate(encoder.getDistance(), setpoint));
        }
    }
}
