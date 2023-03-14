package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private NetworkTable table;
    // degrees
    public double heading;
    // ft
    public double x;
    public double y;
    public double z;
    public double tag_translation_range;
    public double tag_rotation_range;
    // this entry is u128 in rust but gets converted to long via a string value
    public long frame_time;
    // ms
    public static final long max_time_epsilon = 250;
    public Vision() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("vision");
    }

    @Override
    public void periodic() {
        // update network table entries
        heading = table.getEntry("heading").getDouble(Double.MAX_VALUE);
        x = table.getEntry("x_pos").getDouble(Double.MAX_VALUE);
        y = table.getEntry("y_pos").getDouble(Double.MAX_VALUE);
        z = table.getEntry("z_pos").getDouble(Double.MAX_VALUE);
        tag_translation_range = table.getEntry("tag_translation_range").getDouble(Double.MAX_VALUE);
        tag_rotation_range = table.getEntry("tag_rotation_range").getDouble(Double.MAX_VALUE);
        frame_time = Long.parseLong(table.getEntry("frame_time").getString("0"));
    }

    public boolean valid() {
        return (Math.abs(frame_time - System.currentTimeMillis()) < max_time_epsilon);
    }
}
