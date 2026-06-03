package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class CameraWrapper {
    private final String name;
    @SuppressWarnings("unused")
    private final Transform3d robotToCamera;

    private double lastTimestampSeconds = 0;
    private Pose2d lastPoseMT1 = null;
    private Pose2d lastPoseMT2 = null;

    public CameraWrapper(String name, Transform3d robotToCamera) {
        this.name = name;
        this.robotToCamera = robotToCamera;
    }

    public String getName() { return name; }

    public RawFiducial[] getRawFiducials() {
        return LimelightHelpers.getRawFiducials(name);
    }

    public LimelightHelpers.PoseEstimate getPoseEstimateMT1() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    }

    public LimelightHelpers.PoseEstimate getPoseEstimateMT2() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }

    // --- State tracking ---

    public double getLastTimestampSeconds() { return lastTimestampSeconds; }
    public void setLastTimestampSeconds(double ts) { lastTimestampSeconds = ts; }

    public Pose2d getLastPoseMT1() { return lastPoseMT1; }
    public void setLastPoseMT1(Pose2d pose) { lastPoseMT1 = pose; }

    public Pose2d getLastPoseMT2() { return lastPoseMT2; }
    public void setLastPoseMT2(Pose2d pose) { lastPoseMT2 = pose; }
}
