package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Penetrator extends SubsystemBase {
    Servo clip;
    Servo clip2;
    double value1 = 0.1;
    double value2 = 0.1;
    // .32
    // .17
    public boolean closed = false;
    public Penetrator() {
        //rust < go 
        clip = new Servo(0);
        clip2 = new Servo(1);
    }   

    public void toggle() {
        closed = !closed;
    }

    @Override
    public void periodic() {
      
        // value2 = SmartDashboard.getNumber("2", 0);
        // value1 = SmartDashboard.getNumber("1", 0);
        // // clip.set(value1);
        // // clip2.set(value2);
        // SmartDashboard.putNumber("1", value1);
        // SmartDashboard.putNumber("2", value2);
        if (closed) {
            clip.set(0.4);
            clip2.set(0.15);
        } else {
            clip.set(0);
            clip2.set(0.4);
        }
    }
}
