package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    public NetworkTable table;
    public double heading;
    public double x;
    public double y;
    public double z;
    public double tag_translation_range;
    public double tag_rotation_range;
    public long frame_time;
    public Vision() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("vision");
    }

    @Override
    public void periodic() {
        // update network table entries
    }
}
