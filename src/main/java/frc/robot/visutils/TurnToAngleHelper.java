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
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
     * Resolves an AprilTag ID to a field-coordinate (x, y) target.
     *
     * @param tagId the AprilTag ID
     * @return a two-element array {x, y} in blue-origin metres,
     *         or {0, 0} if the tag is not found
     */
    public static double[] resolveTagTarget(int tagId) {
        Optional<Pose3d> pose = getTagPose(tagId);
        if (pose.isPresent()) {
            return new double[] {pose.get().getX(), pose.get().getY()};
        }
        System.err.println("TurnToAngleHelper: Tag ID " + tagId + " not found!");
        return new double[] {0, 0};
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
        // P of 3 keeps heading corrections gentle enough that module
        // angles stay stable when translation and rotation are combined.
        // D of 0.6 damps residual oscillation for a smooth settle.
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

    // ── Aim-while-driving with feedforward ──────────────────────

    /** Heading PID gains used by {@link #createAimPID()}. */
    private static final double kAimP = 3.0;
    private static final double kAimD = 0.6;

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
     * Computes a rotational rate (rad/s) that aims the robot at a
     * field point using PID + translational feedforward.
     *
     * <p>The feedforward anticipates how the bearing changes due to
     * translational motion, so the PID only handles residual error.
     * This produces much smoother module behaviour than a pure-PID
     * {@link SwerveRequest.FieldCentricFacingAngle} when driving
     * and aiming simultaneously.
     *
     * @param targetX        target X (blue-origin, m)
     * @param targetY        target Y (blue-origin, m)
     * @param robotPose      current robot pose
     * @param fieldSpeeds    field-relative chassis speeds
     * @param headingPID     the caller's heading PID (radians)
     * @param maxOmega       maximum angular velocity to clamp to (rad/s)
     * @return clamped rotational rate in rad/s
     */
    public static double computeAimRate(
            double targetX, double targetY,
            Pose2d robotPose, ChassisSpeeds fieldSpeeds,
            PIDController headingPID, double maxOmega) {
        // Desired heading in field frame
        double desiredRad = bearingToPoint(
            targetX, targetY, robotPose).getRadians();
        double currentRad = robotPose.getRotation().getRadians();

        double pidOut = headingPID.calculate(currentRad, desiredRad);
        double ff = aimFeedforward(
            targetX, targetY, robotPose, fieldSpeeds);

        return MathUtil.clamp(pidOut + ff, -maxOmega, maxOmega);
    }
}
