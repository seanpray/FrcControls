package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class combus extends SubsystemBase {
    public double heading;
    public double x;
    public double y;
    public double z;
    public double tag_translation_range;
    public double tag_rotation_range;
    public long frame_time;
    public combus() {}

    @Override
    public void periodic() {
        // update network table entries
    }
}
