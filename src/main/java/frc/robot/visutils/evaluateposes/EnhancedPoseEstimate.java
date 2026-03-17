package frc.robot.visutils.evaluateposes;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.LimelightHelpers.PoseEstimate;
import java.util.Optional;
import java.util.OptionalDouble;


/** Simple helpers to calculate extra info on the pose. */
public class EnhancedPoseEstimate {
    private final PoseEstimate m_visionPoseEstimate;
    private final SwerveDriveState m_driveState;
    private final Optional<Pose2d> m_robotPoseAtSnapTime;

    /** Constructor. */
    public EnhancedPoseEstimate(
        PoseEstimate visionPoseEstimate,
        SwerveDriveState driveState,
        Optional<Pose2d> robotPoseAtSnapTime) {

        // We assume the visionPoseEstimate passed in is valid, as well as the driveState
        // and driveState.Pose.
        if (visionPoseEstimate == null || visionPoseEstimate.pose == null) {
            throw new IllegalArgumentException("visionPoseEstimate and its pose must be non-null");
        }
        if (driveState == null || driveState.Pose == null) {
            throw new IllegalArgumentException("driveState and its Pose must be non-null");
        }

        m_visionPoseEstimate = visionPoseEstimate;
        m_driveState = driveState;
        m_robotPoseAtSnapTime = robotPoseAtSnapTime;
    }

    /** Returns the raw vision pose estimate. */
    public PoseEstimate getVisionPoseEstimate() {
        return m_visionPoseEstimate;
    }

    /** Returns the estimated vision pose as a {@link Pose2d}. */
    public Pose2d getVisionPose2d() {
        return m_visionPoseEstimate.pose;
    }

    /** Returns whether this estimate came from MegaTag2. */
    public boolean isMegatag2() {
        return m_visionPoseEstimate.isMegaTag2;
    }

    //
    // Some helpers to compare pose to the CURRENT robot pose.
    //

    /** Returns the XY distance from the vision pose to the current robot pose. */
    public double getXyDistanceToCurrentRobotPose() {
        return m_visionPoseEstimate.pose
        .getTranslation()
        .getDistance(m_driveState.Pose.getTranslation());
    }

    //
    // Some helpers to compare pose to the PAST robot pose (at camera snap time
    // of this vision pose).
    //

    /** Returns the translation error between the vision pose and robot pose at snap time. */
    public Optional<Translation2d> getVisionPoseErrorAtSnapTime() {
        if (m_robotPoseAtSnapTime.isEmpty()) {
            return Optional.empty();
        }

        return Optional.of(
            m_visionPoseEstimate.pose
                .minus(m_robotPoseAtSnapTime.get())
                .getTranslation());
    }

    /** Returns the snap-time vision pose error magnitude in meters. */
    public OptionalDouble getVisionPoseErrorAtSnapTimeInMeters() {
        Optional<Translation2d> visionPoseErrorAtSnapTime = getVisionPoseErrorAtSnapTime();
        if (visionPoseErrorAtSnapTime.isEmpty()) {
            return OptionalDouble.empty();
        }

        return OptionalDouble.of(visionPoseErrorAtSnapTime.get().getNorm());
    }

    //
    // Helpers to analyze Fiducials (i.e. per-april-tag info).
    //
    // $TODO
}
