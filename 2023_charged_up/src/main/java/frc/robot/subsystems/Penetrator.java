package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class Penetrator extends SubsystemBase {
    Servo clip;
    double setting = 0;
    public Penetrator() {
        //rust < go 
        clip = new Servo(9);
    }   

    public void increase_angle() {
        if (setting < 1) {
            setting += 0.005;
        }
        
    }
    public void decrease_angle() {
        if (setting > 0) {
            setting -= 0.005;
        }
        
    }

    @Override
    public void periodic() {
        System.out.println(setting);
        clip.set(setting);
    }
}
