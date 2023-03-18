package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class Penetrator extends SubsystemBase {
    Servo clip;
    Servo clip2;
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
        if (closed) {
            clip.set(0.1);
            clip2.set(0.35);
        } else {
            clip.set(0.35);
            clip2.set(0.1);
        }
    }
}
