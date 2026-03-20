//
// Ground truth simulator, by Ramen Robotics (9036), 2026.
//
// In simulation, field shows BOTH:
// 1) The estimated robot pose based on odometry - This is the pose that our code
//   "thinks" the robot is at on the field.
// 2) The GROUND TRUTH POSE.  This is a simulation of the actual PHYSICAL robot pose.
//
// Pressing trigger "offsets" the estimated pose, so that you can test how your
// robot code corrects it.
//

package frc.robot.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.visutils.AllianceCalc;
import java.util.function.Consumer;
import java.util.function.Supplier;


/**
 * Simulation helper that tracks the ground truth robot pose
 * independently of odometry drift. This allows testing vision correction
 * by providing ground truth to the simulated cameras.
 */
public class GroundTruthSim implements GroundTruthSimInterface {

    /** Supplier for the current chassis speeds (robot-relative). */
    private final Supplier<ChassisSpeeds> m_speedsSupplier;

    /** Supplier for the current estimated pose from the pose estimator. */
    private final Supplier<Pose2d> m_estimatedPoseSupplier;

    /** Consumer to reset the pose estimator (e.g. drivetrain.resetPose). */
    private final Consumer<Pose2d> m_drivetrainResetPose;

    /** Consumer to notify RobotContainer when pose is reset. */
    private final Consumer<Pose2d> m_poseResetConsumer;

    /** The ground truth pose tracks where the robot actually is in simulation physics. */
    private Pose2d m_groundTruthPose = new Pose2d();

    /** Track accumulated distance for telemetry. */
    private double m_totalDistanceTraveled = 0.0;

    @SuppressWarnings("unused")
    private double m_totalRotation = 0.0;

    /** Last update time for delta calculation. */
    private double m_lastUpdateTime;

    /** Timeout in seconds before cycle resets to beginning. */
    private static final double CYCLE_TIMEOUT_SECONDS = 2.0;

    /**
     * Simulated rightward pull: how far right (meters) the robot drifts
     * for each meter of forward travel. Models real-world drivetrain asymmetry.
     */
    private static final double kRightDriftMetersPerMeterForward = Inches.of(6).in(Meters);

    /**
     * Simulated clockwise rotation: how many radians the robot drifts clockwise
     * for each meter of forward travel. Models real-world drivetrain turn asymmetry.
     */
    private static final double kClockwiseRotationRadiansPerMeterForward = Degrees.of(1).in(Radians);

    /** Whether the simulated rightward pull is currently active. */
    private boolean m_pullRightEnabled = false;

    /** Whether the simulated clockwise rotation drift is currently active. */
    private boolean m_rotateClockwiseEnabled = false;

    private double m_lastCycleTime = 0.0;

    /** Current position in the reset cycle (0-N). */
    private int m_currrentCycleState = 0;

    /**
     * Constructs a GroundTruthSim instance.
     * This class is only intended for use in simulation.
     *
     * @param drivetrain The swerve drivetrain to track and manipulate
     * @param poseResetConsumer Consumer to be called when pose is reset
     *     (e.g., RobotContainer::resetRobotPose)
     * @throws IllegalStateException if called outside of simulation mode
     */
    public GroundTruthSim(
        SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain,
        Consumer<Pose2d> poseResetConsumer) {

        this(
            () -> drivetrain.getState().Speeds,
            () -> drivetrain.getState().Pose,
            drivetrain::resetPose,
            poseResetConsumer);
    }

    /**
     * Constructs a GroundTruthSim instance with fully injectable dependencies.
     * Package-private to allow unit tests to supply deterministic collaborators
     * without depending on vendor hardware classes.
     *
     * @param speedsSupplier Supplies the current robot-relative chassis speeds
     * @param estimatedPoseSupplier Supplies the current estimated pose from the pose estimator
     * @param drivetrainResetPose Consumer to reset the drivetrain's pose estimator
     * @param poseResetConsumer Consumer to be called when pose is reset (e.g. for vision)
     * @throws IllegalStateException if called outside of simulation mode
     */
    GroundTruthSim(
        Supplier<ChassisSpeeds> speedsSupplier,
        Supplier<Pose2d> estimatedPoseSupplier,
        Consumer<Pose2d> drivetrainResetPose,
        Consumer<Pose2d> poseResetConsumer) {

        if (!Robot.isSimulation()) {
            throw new IllegalStateException("GroundTruthSim only instantiated in simulation mode");
        }
        this.m_speedsSupplier = speedsSupplier;
        this.m_estimatedPoseSupplier = estimatedPoseSupplier;
        this.m_drivetrainResetPose = drivetrainResetPose;
        this.m_poseResetConsumer = poseResetConsumer;
        this.m_lastUpdateTime = Utils.getCurrentTimeSeconds();
    }

    /**
     * Updates the ground truth pose by integrating chassis speeds.
     * Call this from Robot.simulationPeriodic().
     */
    public void updateGroundTruthPose() {
        double currentTime = Utils.getCurrentTimeSeconds();
        double deltaTime = currentTime - m_lastUpdateTime;
        m_lastUpdateTime = currentTime;

        ChassisSpeeds speeds = m_speedsSupplier.get();

        // Calculate how much the robot moved this timestep
        double dx = speeds.vxMetersPerSecond * deltaTime;
        double dy = speeds.vyMetersPerSecond * deltaTime;
        double dtheta = speeds.omegaRadiansPerSecond * deltaTime;

        // Simulate rightward pull: robot drifts right proportional to forward movement only.
        // (in robot frame, right = negative Y; no effect when driving backwards)
        if (m_pullRightEnabled) {
            dy -= Math.max(dx, 0) * kRightDriftMetersPerMeterForward;
        }

        // Simulate clockwise rotation drift: robot turns CW proportional to forward movement only.
        // (clockwise = negative dtheta; no effect when driving backwards)
        if (m_rotateClockwiseEnabled) {
            dtheta -= Math.max(dx, 0) * kClockwiseRotationRadiansPerMeterForward;
        }

        double distanceThisStep = Math.hypot(dx, dy);
        double rotationThisStep = Math.abs(dtheta);

        m_totalDistanceTraveled += distanceThisStep;
        m_totalRotation += rotationThisStep;

        // Update the ground truth pose (using field-relative velocities)
        // Rotate the robot-relative velocity by the current heading to get field-relative
        double cos = Math.cos(m_groundTruthPose.getRotation().getRadians());
        double sin = Math.sin(m_groundTruthPose.getRotation().getRadians());
        double fieldDx = dx * cos - dy * sin;
        double fieldDy = dx * sin + dy * cos;

        m_groundTruthPose = new Pose2d(
            m_groundTruthPose.getX() + fieldDx,
            m_groundTruthPose.getY() + fieldDy,
            m_groundTruthPose.getRotation().plus(new Rotation2d(dtheta))
        );
    }

    /**
     * Gets the ground truth simulated pose (where the robot actually is based on physics).
     * Use this for PhotonVision simulation so cameras see the correct AprilTags.
     *
     * @return The ground truth pose of the robot in simulation
     */
    @Override
    public Pose2d getGroundTruthPose() {
        return m_groundTruthPose;
    }

    /**
     * Resets both the ground truth pose.
     *
     * @param pose The pose to reset both ground truth and drivetrain to
     */
    @Override
    public void resetGroundTruthPoseForSim(Pose2d pose) {
        m_groundTruthPose = pose;
        m_totalDistanceTraveled = 0.0;
        m_totalRotation = 0.0;
    }

    /**
     * Resets both poses to a PathPlanner auto starting pose, optionally flipping based on alliance.
     * PathPlanner paths are designed for blue alliance origin.
     *
     * @param blueAlliancePose The pose as defined in PathPlanner (blue alliance origin)
     * @param useCorrectTeamSide If true, places robot on correct alliance side (flips for red).
     *                           If false, places robot on the wrong side of the field.
     */
    public void resetAllPosesToSelectedAutoPos(
        Pose2d blueAlliancePose,
        boolean useCorrectTeamSide) {

        boolean shouldFlipPose = AllianceCalc.isRedAlliance();
        if (!useCorrectTeamSide) {
            shouldFlipPose = !shouldFlipPose;
        }

        Pose2d pose = shouldFlipPose
            ? AllianceCalc.flipFieldPose(blueAlliancePose)
            : blueAlliancePose;

        // Trigger robot pose reset
        m_poseResetConsumer.accept(pose);
    }

    /**
     * Cycles through reset positions each time it's called.
     * Cycle order:
     *   0: Auto starting pose on correct alliance side
     *   1: Auto starting pose on wrong alliance side
     *   2: Origin on correct alliance side
     *   3: Origin on wrong alliance side
     *
     * <p>If CYCLE_TIMEOUT_SECONDS elapses between calls, restarts from 0.
     *
     * @param blueAlliancePose The auto starting pose (blue alliance origin)
     */
    @Override
    public void cycleResetPosition(Pose2d blueAlliancePose) {
        double currentTime = Utils.getCurrentTimeSeconds();

        // Reset cycle if timeout elapsed
        if (currentTime - m_lastCycleTime > CYCLE_TIMEOUT_SECONDS) {
            m_currrentCycleState = 0;
        }
        m_lastCycleTime = currentTime;

        // Execute based on current cycle state
        switch (m_currrentCycleState) {
            case 0 -> resetAllPosesToSelectedAutoPos(blueAlliancePose, true);
            case 1 -> resetAllPosesToSelectedAutoPos(blueAlliancePose, false);
            case 2 -> resetAllPosesToSelectedAutoPos(new Pose2d(), true);
            case 3 -> resetAllPosesToSelectedAutoPos(new Pose2d(), false);
            default -> throw new IllegalStateException(
                "Invalid cycle state: " + m_currrentCycleState);
        }

        // Advance to next state
        m_currrentCycleState = (m_currrentCycleState + 1) % 4;
    }

    /**
     * Introduces simulated odometry drift by offsetting the pose estimator.
     * The "true" pose remains unchanged, but the pose estimator
     * is reset to a drifted position. Vision should then correct this drift.
     *
     * @param translationOffsetMeters How far to offset the estimated position (meters)
     * @param rotationOffsetDegrees How far to offset the estimated heading (degrees)
     */
    @Override
    public void injectDriftToPoseEstimate(double xOffsetFrontBack, double yOffsetLeftRight, double rotationOffsetDegrees) {
        // Get current estimated pose
        Pose2d currentPose = m_estimatedPoseSupplier.get();

        // Apply offsets in the robot's local frame (x = forward, y = left)
        Pose2d driftedPose = currentPose.transformBy(
            new Transform2d(xOffsetFrontBack, yOffsetLeftRight, Rotation2d.fromDegrees(rotationOffsetDegrees))
        );

        // Reset the pose estimator to the drifted position
        // The ground truth pose remains at the actual position
        m_drivetrainResetPose.accept(driftedPose);
    }

    /**
     * Offsets the ground truth pose while leaving the pose estimator unchanged.
     * This moves where the robot "actually is" in simulation (and thus where
     * cameras see AprilTags), without touching the odometry estimate.
     *
     * @param xOffsetFrontBack Forward/back offset in the robot's local frame (meters)
     * @param yOffsetLeftRight Left/right offset in the robot's local frame (meters)
     * @param rotationOffsetDegrees Heading offset (degrees)
     */
    @Override
    public void injectDriftToGroundTruth(double xOffsetFrontBack, double yOffsetLeftRight, double rotationOffsetDegrees) {
        // Apply offsets in the robot's local frame (x = forward, y = left)
        m_groundTruthPose = m_groundTruthPose.transformBy(
            new Transform2d(xOffsetFrontBack, yOffsetLeftRight, Rotation2d.fromDegrees(rotationOffsetDegrees))
        );
        // The pose estimator is left untouched
    }

    /**
     * Enables or disables simulated rightward pull during forward motion.
     *
     * @param enabled true to enable pull-right, false to disable
     */
    @Override
    public void enablePullRight(boolean enabled) {
        m_pullRightEnabled = enabled;
    }

    /**
     * Enables or disables simulated clockwise rotation drift during turning.
     *
     * @param enabled true to enable clockwise rotation drift, false to disable
     */
    @Override
    public void enableRotateClockwise(boolean enabled) {
        m_rotateClockwiseEnabled = enabled;
    }

    /**
     * Publishes simulation telemetry to SmartDashboard.
     * Call this from Robot.simulationPeriodic().
     */
    public void publishTelemetry() {
        SmartDashboard.putNumber("Sim/GroundTruth/X", m_groundTruthPose.getX());
        SmartDashboard.putNumber("Sim/GroundTruth/Y", m_groundTruthPose.getY());
        SmartDashboard.putNumber(
            "Sim/GroundTruth/RotationDeg",
            m_groundTruthPose.getRotation().getDegrees());

        Pose2d estimatedPose = m_estimatedPoseSupplier.get();
        SmartDashboard.putNumber("Sim/EstimatedPose/X", estimatedPose.getX());
        SmartDashboard.putNumber("Sim/EstimatedPose/Y", estimatedPose.getY());
        SmartDashboard.putNumber(
            "Sim/EstimatedPose/RotationDeg",
            estimatedPose.getRotation().getDegrees());

        double poseError = m_groundTruthPose
            .getTranslation()
            .getDistance(estimatedPose.getTranslation());
        double headingError = Math.abs(
            m_groundTruthPose.getRotation().minus(estimatedPose.getRotation()).getDegrees());
        SmartDashboard.putNumber("Sim/PoseErrorMeters", poseError);
        SmartDashboard.putNumber("Sim/HeadingErrorDeg", headingError);
        SmartDashboard.putNumber("Sim/TotalDistanceTraveled", m_totalDistanceTraveled);
    }

    /**
     * Updates ground truth pose, and publishes telemetry.
     * Call this from Robot.simulationPeriodic().
     */
    @Override
    public void simulationPeriodic() {
        // Ground truth pose is integrated at high frequency via updateGroundTruthPose()
        // (registered as setHighFreqSimCallback on CommandSwerveDrivetrain).
        // Here we only publish telemetry at the standard 50 Hz robot loop rate.
        publishTelemetry();
    }
}
