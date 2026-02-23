package frc.robot.visutils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

import frc.robot.visutils.VisionKalmanFilter;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/** Basic logging to dashboard. */
@SuppressWarnings("LineLength")
public class BasicInfoDashboard {
    private final Pigeon2 m_pigeon;
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_drivetrain;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Gyro state and robot heading. */
    private final NetworkTable m_basicInfoTable = inst.getTable("BasicInfo");
    private final DoublePublisher m_driveHeadingDegrees = m_basicInfoTable.getDoubleTopic("HeadingDegrees").publish();
    private final DoublePublisher m_gyroYawDegrees = m_basicInfoTable.getDoubleTopic("GyroYawDegrees").publish();

    /* Other basic info. */
    private final StringPublisher AllianceWithBlueFallback = m_basicInfoTable.getStringTopic("AllianceWithBlueFallback").publish();
    private final StringPublisher AllianceNT = m_basicInfoTable.getStringTopic("AllianceNT").publish();
    private final BooleanPublisher IsDSAttached = m_basicInfoTable.getBooleanTopic("IsDSAttached").publish();
    private final BooleanPublisher IsFMSAttached = m_basicInfoTable.getBooleanTopic("IsFMSAttached").publish();
    private final StringPublisher OperatorForwardDirectionDegrees = m_basicInfoTable.getStringTopic("OperatorForwardDirectionDegrees").publish();
    private final StringPublisher DebugCalculatedForwardDirection = m_basicInfoTable.getStringTopic("DebugCalculatedForwardDirection").publish();
    private final DoublePublisher m_visionConfidence = m_basicInfoTable.getDoubleTopic("VisionConfidence").publish();
    private final BooleanPublisher m_oneLocked = m_basicInfoTable.getBooleanTopic("OneLocked").publish();
    private final BooleanPublisher m_multiLocked = m_basicInfoTable.getBooleanTopic("MultiLocked").publish();
    private final DoublePublisher m_visionTx = m_basicInfoTable.getDoubleTopic("VisionTx").publish();
    private final StringPublisher m_targetList = m_basicInfoTable.getStringTopic("TargetList").publish();

    /** Bidirectional toggle for enabling/disabling vision measurement injection. */
    private final BooleanEntry m_visionEnabled =
        m_basicInfoTable.getBooleanTopic("VisionEnabled").getEntry(Constants.VisionConstants.kVisionEnabledDefault);

    /* Vision Kalman filter outputs */
    private final DoubleArrayPublisher m_visionKalmanPose =
        m_basicInfoTable.getDoubleArrayTopic("VisionKalmanPose").publish();
    private final BooleanPublisher m_visionKalmanConverged =
        m_basicInfoTable.getBooleanTopic("VisionKalmanConverged").publish();
    private final BooleanPublisher m_visionKalmanActive =
        m_basicInfoTable.getBooleanTopic("VisionKalmanActive").publish();
    private final BooleanPublisher m_visionKalmanIsRobotStill =
        m_basicInfoTable.getBooleanTopic("VisionKalmanIsRobotStill").publish();
    private final StringPublisher m_visionKalmanSecondsStill =
        m_basicInfoTable.getStringTopic("VisionKalmanSecondsStill").publish();

    private DoubleSupplier m_visionConfidenceSupplier = null;
    private IntSupplier m_numLockedTagsSupplier = null;
    private DoubleSupplier m_txSupplier = null;
    private Supplier<List<Integer>> m_targetListSupplier = null;
    private Supplier<VisionKalmanFilter> m_visionKalmanSupplier = null;
    private BooleanSupplier m_isRobotMotionlessSupplier = null;
    private DoubleSupplier m_secondsStillSupplier = null;

    /** When true, vision is forcibly disabled regardless of the dashboard toggle. */
    private boolean m_forceDisableVision = false;

    // Debouncers for locked indicators (prevents flickering)
    private static final double kLockedDebounceSeconds = 0.25;
    private final Debouncer m_oneLockedDebouncer =
        new Debouncer(kLockedDebounceSeconds, Debouncer.DebounceType.kFalling);
    private final Debouncer m_multiLockedDebouncer =
        new Debouncer(kLockedDebounceSeconds, Debouncer.DebounceType.kFalling);
    private final Debouncer m_targetListDebouncer =
        new Debouncer(kLockedDebounceSeconds, Debouncer.DebounceType.kFalling);
    private String m_lastNonEmptyTargetList = "";

    /**
     * Constructs a BasicInfoDashboard.
     *
     * @param drivetrain The swerve drivetrain to get info from
     */
    public BasicInfoDashboard(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain) {
        m_drivetrain = drivetrain;
        m_pigeon = drivetrain.getPigeon2();

        // Publish the default value so the toggle appears in Elastic immediately.
        m_visionEnabled.set(Constants.VisionConstants.kVisionEnabledDefault);
    }

    /**
     * Returns whether vision measurements should be injected into odometry.
     * Returns false if vision is force-disabled, otherwise reads the dashboard toggle.
     *
     * @return true if vision is enabled
     */
    public boolean isVisionEnabled() {
        if (m_forceDisableVision) {
            return false;
        }
        return m_visionEnabled.get(Constants.VisionConstants.kVisionEnabledDefault);
    }

    /**
     * Force-disables or re-enables vision measurement injection.
     * When force-disabled, {@link #isVisionEnabled()} always returns false
     * regardless of the dashboard toggle.
     *
     * @param disable true to force-disable vision, false to resume normal behavior
     */
    public void forceDisableVision(boolean disable) {
        m_forceDisableVision = disable;
    }

    /**
     * Sets the suppliers for querying basic vision information.
     *
     * @param confidenceSupplier    A DoubleSupplier returning confidence 0-100
     * @param numLockedTagsSupplier An IntSupplier returning the locked tag count
     * @param txSupplier            A DoubleSupplier returning tx in degrees
     * @param targetListSupplier    A Supplier returning comma-separated tag IDs
     * @param isMotionlessSupplier  A BooleanSupplier returning true when robot is motionless
     * @param secondsStillSupplier  A DoubleSupplier returning seconds the robot has been still
     */
    public void setVisionDependencies(
            DoubleSupplier confidenceSupplier,
            IntSupplier numLockedTagsSupplier,
            DoubleSupplier txSupplier,
            Supplier<List<Integer>> targetListSupplier,
            BooleanSupplier isMotionlessSupplier,
            DoubleSupplier secondsStillSupplier) {
        m_visionConfidenceSupplier = confidenceSupplier;
        m_numLockedTagsSupplier = numLockedTagsSupplier;
        m_txSupplier = txSupplier;
        m_targetListSupplier = targetListSupplier;
        m_isRobotMotionlessSupplier = isMotionlessSupplier;
        m_secondsStillSupplier = secondsStillSupplier;
    }

    /**
     * Sets the supplier for the vision Kalman filter.
     *
     * @param supplier A Supplier returning the VisionKalmanFilter instance
     */
    public void setVisionKalmanSupplier(Supplier<VisionKalmanFilter> supplier) {
        m_visionKalmanSupplier = supplier;
    }

    private static String targetListToString(List<Integer> targets) {
        return targets.stream()
            .map(String::valueOf)
            .collect(Collectors.joining(", "));
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    /* This is not the forward direction as seen by the drivetrain.  Instead, this is what
       its supposed to be, based on the alliance color.  The CommandSwerveDrivetrain has
       some funny logic in how it calculates it, so we display the correct value
       in Dashboard for easier debugging.
    */
    private String getDebugCalculatedForwardString() {
        if (isRedAlliance()) {
            return "WEST";
        } else {
            return "EAST";
        }
    }

    /** Called after each Robot Periodic. */
    public void update() {
        /* Gyro and robot heading */
        m_driveHeadingDegrees.set(m_drivetrain.getState().Pose.getRotation().getDegrees());
        m_gyroYawDegrees.set(m_pigeon.getYaw().getValueAsDouble());

        /* Publish other basic info */
        AllianceWithBlueFallback.set(isRedAlliance() ? "Red" : "Blue");
        AllianceNT.set(DriverStation.getAlliance().map(Object::toString).orElse("UNKNOWN"));
        IsDSAttached.set(DriverStation.isDSAttached());
        IsFMSAttached.set(DriverStation.isFMSAttached());

        /* Publish operator forward direction
            If the operator is in the Blue Alliance Station, this should be 0 degrees (EAST).
            If the operator is in the Red Alliance Station, this should be 180 degrees (WEST).
        */
        double forwardDegrees = m_drivetrain.getOperatorForwardDirection().getDegrees();
        String screenDirection = (Math.abs(forwardDegrees) < 90) ? "EAST" : "WEST";
        OperatorForwardDirectionDegrees.set(screenDirection);

        DebugCalculatedForwardDirection.set(getDebugCalculatedForwardString());

        /* Publish vision confidence */
        if (m_visionConfidenceSupplier != null) {
            m_visionConfidence.set(m_visionConfidenceSupplier.getAsDouble());
        }
        if (m_numLockedTagsSupplier != null) {
            int numTags = m_numLockedTagsSupplier.getAsInt();
            m_oneLocked.set(m_oneLockedDebouncer.calculate(numTags >= 1));
            m_multiLocked.set(m_multiLockedDebouncer.calculate(numTags >= 2));
        }
        if (m_txSupplier != null) {
            m_visionTx.set(m_txSupplier.getAsDouble());
        }
        if (m_targetListSupplier != null) {
            List<Integer> targets = m_targetListSupplier.get();
            boolean hasTargets = !targets.isEmpty();
            if (hasTargets) {
                m_lastNonEmptyTargetList = targetListToString(targets);
            }
            // Debounce the "has targets" state - when it goes false, delay before showing empty
            boolean showTargets = m_targetListDebouncer.calculate(hasTargets);
            m_targetList.set(showTargets ? m_lastNonEmptyTargetList : "");
        }

        /* Publish vision Kalman filter state */
        if (m_visionKalmanSupplier != null) {
            VisionKalmanFilter filter = m_visionKalmanSupplier.get();
            boolean isActive = filter.isInitialized();
            m_visionKalmanActive.set(isActive);

            if (isActive) {
                Pose2d pose = filter.getEstimate();
                m_visionKalmanPose.set(new double[] {
                    pose.getX(),
                    pose.getY(),
                    pose.getRotation().getDegrees()
                });
                m_visionKalmanConverged.set(filter.hasConverged());
            } else {
                m_visionKalmanPose.set(new double[] {0, 0, 0});
                m_visionKalmanConverged.set(false);
            }
        }

        /* Publish robot motionless state for Kalman filter */
        if (m_isRobotMotionlessSupplier != null) {
            m_visionKalmanIsRobotStill.set(m_isRobotMotionlessSupplier.getAsBoolean());
        }
        if (m_secondsStillSupplier != null) {
            double seconds = m_secondsStillSupplier.getAsDouble();
            m_visionKalmanSecondsStill.set(String.format("%.1f", seconds));
        }
    }
}
