package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.LimelightHelpers;

public class Limelight {

    private final NetworkTable table =
        NetworkTableInstance.getDefault().getTable("limelight");

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public double getTX() {
        return table.getEntry("tx").getDouble(0);
    }

    public double getTY() {
        return table.getEntry("ty").getDouble(0);
    }

    public double getDistanceMeters() {
        return LimelightHelpers
            .getTargetPose3d_RobotSpace("limelight")
            .getTranslation()
            .getNorm();
    }
}   