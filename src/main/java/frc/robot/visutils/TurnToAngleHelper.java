package frc.robot.visutils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.Optional;
import java.util.OptionalDouble;

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
     * Resolves an AprilTag ID to a field-coordinate target.
     *
     * @param tagId the AprilTag ID
     * @return the tag position in blue-origin metres,
     *         or empty if the tag is not found
     */
    public static Optional<Translation2d> resolveTagTarget(int tagId) {
        if (tagId == -1) {
            return Optional.empty();
        }

        Optional<Pose3d> pose = getTagPose(tagId);
        if (pose.isPresent()) {
            return Optional.of(new Translation2d(pose.get().getX(), pose.get().getY()));
        }
        System.err.println("TurnToAngleHelper: Tag ID " + tagId + " not found!");
        return Optional.empty();
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
        SwerveRequest.FieldCentricFacingAngle request =
            new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(DriveRequestType.Velocity);
        configureFacingAngle(request);
        return request;
    }

    // ── Aim-while-driving with feedforward ──────────────────────

    /** Heading PID gains used by {@link #createAimPID()}. */
    private static final double kAimP = 5.0;
    private static final double kAimD = 0.4;

    /**
     * Maximum distance (metres) at which we'll aim at a remembered tag.
     * Beyond this the target is likely stale / on the far side of the field.
     */
    private static final double kMaxAimDistanceMeters = 5.0;

    /**
     * Creates a WPILib PID controller pre-configured for heading
     * aim-while-driving. Caller owns the instance.
     *
     * @return a continuous-input heading PID (radians)
     */
    public static PIDController createAimPID() {
        PIDController pid = new PIDController(kAimP, 0, kAimD);
        pid.enableContinuousInput(-Math.PI, Math.PI);
        return pid;
    }

    /**
     * Computes the rotational feedforward needed to track a fixed
     * field point while translating.
     *
     * <p>This is the analytical derivative of
     * {@code atan2(dy, dx)} with respect to time:
     * <pre>
     *   ff = (dx·vy − dy·vx) / (dx² + dy²)
     * </pre>
     * where (dx, dy) is the vector from the robot to the target
     * and (vx, vy) are the robot's field-relative velocities.
     *
     * @param targetX     target X (blue-origin, m)
     * @param targetY     target Y (blue-origin, m)
     * @param robotPose   current robot pose
     * @param fieldSpeeds field-relative chassis speeds
     * @return rotational feedforward in rad/s
     */
    public static double aimFeedforward(
            double targetX, double targetY,
            Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();
        double distSq = dx * dx + dy * dy;
        // Guard against divide-by-zero when sitting on the target
        if (distSq < 0.01) {
            return 0.0;
        }
        double vx = fieldSpeeds.vxMetersPerSecond;
        double vy = fieldSpeeds.vyMetersPerSecond;
        return (dx * vy - dy * vx) / distSq;
    }

    /**
     * Resolves a tag target and computes the aim rate in one step,
     * handling target validity and field-speed conversion.
     *
     * <p>Call this from the drive loop when the driver is holding
     * the aim button.  Returns empty if no valid target is available;
     * otherwise returns the clamped rotational rate.
     *
     * <p><b>Note:</b> The caller is responsible for resetting
     * {@code headingPID} on the aiming-start transition edge.
     *
     * @param tagId          the last-seen AprilTag ID (−1 = none)
     * @param robotPose      current robot pose
     * @param robotSpeeds    robot-relative chassis speeds
     * @param headingPID     the caller's heading PID (radians)
     * @param maxOmega       maximum angular velocity to clamp to (rad/s)
     * @return the aim rate in rad/s, or empty if the target is invalid
     */
    public static OptionalDouble computeAimRateForTag(
            int tagId,
            Pose2d robotPose,
            ChassisSpeeds robotSpeeds,
            PIDController headingPID,
            double maxOmega) {

        Optional<Translation2d> target = resolveTagTarget(tagId);
        if (target.isEmpty()) {
            return OptionalDouble.empty();
        }

        // Ignore stale targets on the far side of the field
        double dist = robotPose.getTranslation()
            .getDistance(target.get());
        if (dist > kMaxAimDistanceMeters) {
            return OptionalDouble.empty();
        }

        ChassisSpeeds fieldSpeeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation());

        return OptionalDouble.of(
            computeAimRate(target.get(), robotPose, fieldSpeeds, headingPID, maxOmega));
    }

    /**
     * Computes a rotational rate (rad/s) that aims the robot at a
     * field point using PID + translational feedforward.
     *
     * <p>The feedforward anticipates how the bearing changes due to
     * translational motion, so the PID only handles residual error.
     * This produces much smoother module behaviour than a pure-PID
     * {@link SwerveRequest.FieldCentricFacingAngle} when driving
     * and aiming simultaneously.
     *
     * @param target         target position (blue-origin, m)
     * @param robotPose      current robot pose
     * @param fieldSpeeds    field-relative chassis speeds
     * @param headingPID     the caller's heading PID (radians)
     * @param maxOmega       maximum angular velocity to clamp to (rad/s)
     * @return clamped rotational rate in rad/s
     */
    public static double computeAimRate(
            Translation2d target,
            Pose2d robotPose, ChassisSpeeds fieldSpeeds,
            PIDController headingPID, double maxOmega) {
        // Desired heading in field frame
        double desiredRad = bearingToPoint(
            target.getX(), target.getY(), robotPose).getRadians();
        double currentRad = robotPose.getRotation().getRadians();

        double pidOut = headingPID.calculate(currentRad, desiredRad);
        double ff = aimFeedforward(
            target.getX(), target.getY(), robotPose, fieldSpeeds);

        return MathUtil.clamp(pidOut + ff, -maxOmega, maxOmega);
    }
}
