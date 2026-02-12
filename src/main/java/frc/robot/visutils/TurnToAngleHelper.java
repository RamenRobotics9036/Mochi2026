package frc.robot.visutils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

/**
 * Static utility methods for computing bearings to AprilTags and converting
 * between field-frame and operator-perspective angles.
 *
 * <p>Designed to be shared by any command or lambda that needs to aim
 * the robot at a field element (e.g. {@code RotateToTargetCommand},
 * the default drive command's POV-UP lock-on, etc.).
 */
public final class TurnToAngleHelper {

    /** Shared field layout — loaded once, immutable. */
    private static final AprilTagFieldLayout kFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private TurnToAngleHelper() {
    }

    /**
     * Returns the 3-D pose of an AprilTag from the field layout, if it exists.
     *
     * @param tagId the AprilTag ID
     * @return the tag's pose, or empty if the ID is invalid
     */
    public static Optional<Pose3d> getTagPose(int tagId) {
        return kFieldLayout.getTagPose(tagId);
    }

    /**
     * Computes the field-frame bearing from a robot pose to an AprilTag.
     *
     * <p>The returned angle is in the blue-origin field frame
     * (0° = toward the red alliance wall).
     *
     * @param tagId     the AprilTag ID to aim at
     * @param robotPose the robot's current field-relative pose
     * @return the bearing as a {@link Rotation2d}, or {@link Rotation2d#kZero}
     *         if the tag ID is not found
     */
    public static Rotation2d bearingToTag(int tagId, Pose2d robotPose) {
        Optional<Pose3d> tagPose = kFieldLayout.getTagPose(tagId);
        if (tagPose.isEmpty()) {
            return Rotation2d.kZero;
        }
        double dx = tagPose.get().getX() - robotPose.getX();
        double dy = tagPose.get().getY() - robotPose.getY();
        return new Rotation2d(dx, dy);
    }

    /**
     * Computes the field-frame bearing from a robot pose to an arbitrary
     * point on the field.
     *
     * @param targetX   the target X coordinate (blue-origin, metres)
     * @param targetY   the target Y coordinate (blue-origin, metres)
     * @param robotPose the robot's current field-relative pose
     * @return the bearing as a {@link Rotation2d}
     */
    public static Rotation2d bearingToPoint(
            double targetX, double targetY, Pose2d robotPose) {
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();
        return new Rotation2d(dx, dy);
    }

    /**
     * Converts an absolute field-frame angle (0° = toward red wall) into the
     * operator-perspective frame used by
     * {@link SwerveRequest.FieldCentricFacingAngle}.
     *
     * <p>On blue alliance the operator forward is 0° so this is a no-op.
     * On red alliance the operator forward is 180°, so this subtracts 180°.
     * The result is automatically normalized to (−180°, 180°] by
     * {@link Rotation2d}, so wraparound is handled correctly.
     *
     * @param fieldAngle         the angle in field coordinates
     * @param operatorForwardDir the current operator forward direction
     *                           (from {@code drivetrain.getOperatorForwardDirection()})
     * @return the equivalent angle in operator-perspective coordinates
     */
    public static Rotation2d toOperatorFrame(
            Rotation2d fieldAngle, Rotation2d operatorForwardDir) {
        return fieldAngle.minus(operatorForwardDir);
    }

    /**
     * Computes the bearing from a robot pose to an arbitrary field point
     * and converts it to the operator-perspective frame in one step.
     *
     * <p>Equivalent to
     * {@code toOperatorFrame(bearingToPoint(targetX, targetY, robotPose),
     * operatorForwardDir)}.
     *
     * @param targetX          target X coordinate (blue-origin, metres)
     * @param targetY          target Y coordinate (blue-origin, metres)
     * @param robotPose        the robot's current field-relative pose
     * @param operatorForwardDir the current operator forward direction
     * @return the bearing in operator-perspective coordinates
     */
    public static Rotation2d bearingToPointInOperatorFrame(
            double targetX, double targetY,
            Pose2d robotPose, Rotation2d operatorForwardDir) {
        return toOperatorFrame(
            bearingToPoint(targetX, targetY, robotPose),
            operatorForwardDir);
    }

    /**
     * Configures a {@link SwerveRequest.FieldCentricFacingAngle} with
     * standard heading-PID tuning and continuous-input wrapping.
     *
     * <p>Call this once when the request is created so every consumer
     * uses identical gains.
     *
     * @param request the request to configure (mutated in place)
     */
    public static void configureFacingAngle(
            SwerveRequest.FieldCentricFacingAngle request) {
        // P of 5 keeps corrections within the swerve's physical angular-rate
        // limit (~4.7 rad/s) for typical aiming errors, avoiding saturation
        // and the overshoot-oscillation cycle that a higher P causes.
        // D of 0.4 damps residual oscillation without sluggish feel.
        request.HeadingController.setPID(5, 0, 0.4);
        // Shortest-path across the ±180° wrap.
        request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Creates a new {@link SwerveRequest.FieldCentricFacingAngle} that is
     * pre-configured with standard heading-PID tuning and velocity drive mode.
     *
     * @return a ready-to-use facing-angle request
     */
    public static SwerveRequest.FieldCentricFacingAngle createFacingAngleRequest() {

        // NOTE: Normally, we would use VelocityControl for smoother driving, since
        // the CTRE motors are then doing PID themselves.  BUT, OpenLoopVoltage works
        // better here, since FieldCentricFacingAngle is also implementing a
        // separate PID that is constantly adjusting to a new field angle.
        // These two PIDs were fighting each-other, so now we dont use closed
        // loop for the drive motors when driving and aiming.
        SwerveRequest.FieldCentricFacingAngle request =
            new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        configureFacingAngle(request);
        return request;
    }
}
