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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.botconfig.RobotIdentity;
import java.util.List;
import java.util.OptionalDouble;
import java.util.function.Supplier;
import java.util.stream.Collectors;


/**
 * Basic logging to dashboard.
 * The pattern for dashboards is this: This dashboard doesn't know how to query anything out of
 * the various subsystems, since it has no knowledge of them.  Instead, it just:
 * 1) Creates a bunch of NetworkTable entries representing each control in the dashboard, during
 *    construction.
 * 2) Exposes public methods for updating each dashboard value
 *
 * The Dashboard-object is passed around to the various subsystems, each of which calls the
 * relevent 'update' methods.
 *
 * Note that each update-method in this class only updates a local variable, and the actual
 * NetworkTable entries are only updated in the 'update()' method at the end.  This is to avoid
 * writing to NetworkTables more often than necessary.
*/
@SuppressWarnings("LineLength")
public class BasicInfoDashboard {
    private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();

    /* Gyro state and robot heading. */
    private final NetworkTable m_basicInfoTable = m_inst.getTable("BasicInfo");
    private final DoublePublisher m_driveHeadingDegrees = m_basicInfoTable.getDoubleTopic("HeadingDegrees").publish();
    private final DoublePublisher m_gyroYawDegrees = m_basicInfoTable.getDoubleTopic("GyroYawDegrees").publish();

    /* Vision Kalman filter outputs */
    private final DoubleArrayPublisher m_visionKalmanPose = m_basicInfoTable.getDoubleArrayTopic("VisionKalmanPose").publish();
    private final BooleanPublisher m_visionKalmanConverged = m_basicInfoTable.getBooleanTopic("VisionKalmanConverged").publish();
    private final BooleanPublisher m_visionKalmanActive = m_basicInfoTable.getBooleanTopic("VisionKalmanActive").publish();
    private final BooleanPublisher m_visionKalmanIsRobotStill = m_basicInfoTable.getBooleanTopic("VisionKalmanIsRobotStill").publish();
    private final StringPublisher m_visionKalmanSecondsStill = m_basicInfoTable.getStringTopic("VisionKalmanSecondsStill").publish();

    /* Other basic info. */
    private final StringPublisher m_allianceWithBlueFallback = m_basicInfoTable.getStringTopic("AllianceWithBlueFallback").publish();
    private final StringPublisher m_allianceNt = m_basicInfoTable.getStringTopic("AllianceNT").publish();
    private final BooleanPublisher m_isDsAttached = m_basicInfoTable.getBooleanTopic("IsDSAttached").publish();
    private final BooleanPublisher m_isFmsAttached = m_basicInfoTable.getBooleanTopic("IsFMSAttached").publish();
    private final StringPublisher m_operatorForwardDirectionDegrees = m_basicInfoTable.getStringTopic("OperatorForwardDirectionDegrees").publish();
    private final StringPublisher m_debugCalculatedForwardDirection = m_basicInfoTable.getStringTopic("DebugCalculatedForwardDirection").publish();
    private final DoublePublisher m_visionConfidence = m_basicInfoTable.getDoubleTopic("VisionConfidence").publish();
    private final BooleanPublisher m_oneLocked = m_basicInfoTable.getBooleanTopic("OneLocked").publish();
    private final BooleanPublisher m_multiLocked = m_basicInfoTable.getBooleanTopic("MultiLocked").publish();
    private final BooleanPublisher m_isMt2 = m_basicInfoTable.getBooleanTopic("IsMt2").publish();
    private final DoublePublisher m_visionTx = m_basicInfoTable.getDoubleTopic("VisionTx").publish();
    private final StringPublisher m_targetList = m_basicInfoTable.getStringTopic("TargetList").publish();
    private final DoublePublisher m_visErrorCentimeters = m_basicInfoTable.getDoubleTopic("VisErrorCentimeters").publish();
    private final DoublePublisher m_visErrorMultiTagCentimeters = m_basicInfoTable.getDoubleTopic("VisErrorMultiTagCentimeters").publish();

    // Subsystems passed in
    private final Pigeon2 m_pigeon;
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_drivetrain;

    /** Bidirectional toggle for enabling/disabling vision measurement injection. */
    private final BooleanEntry m_visionEnabled;
    private final BotConfigInterface m_configInterface;

    private double m_cachedVisionConfidence = 0.0;
    private boolean m_cachedHasTargetLock = false;
    private boolean m_cachedHasMultiTagLock = false;
    private boolean m_cachedIsLatestMt2 = false;
    private double m_cachedTx = 0.0;
    private List<Integer> m_cachedTargetList = List.of();
    private OptionalDouble m_cachedVisionError = OptionalDouble.empty();
    private boolean m_cachedIsMotionless = false;
    private double m_cachedSecondsStill = 0.0;
    private Supplier<VisionKalmanFilter> m_visionKalmanSupplier = null;

    /** When true, vision is forcibly disabled regardless of the dashboard toggle. */
    private boolean m_forceDisableVision = false;

    /** Bundles a VisionHeartBeat with its NT publisher for one camera. */
    private static class CameraMonitor {
        final VisionHeartBeat heartbeat;
        final StringPublisher status;

        CameraMonitor(String cameraName, String networkTableKeyName, NetworkTable parent) {
            heartbeat = new VisionHeartBeat(cameraName);
            status = parent.getSubTable(networkTableKeyName).getStringTopic("Status").publish();
        }
    }

    /** Per-camera heartbeat monitors. */
    private final List<CameraMonitor> m_cameraMonitors;

    // Debouncers for locked indicators (prevents flickering)
    private static final double kLockedDebounceSeconds = 0.25;
    private final Debouncer m_oneLockedDebouncer =
        new Debouncer(kLockedDebounceSeconds, Debouncer.DebounceType.kFalling);
    private final Debouncer m_multiLockedDebouncer =
        new Debouncer(kLockedDebounceSeconds, Debouncer.DebounceType.kFalling);
    private final Debouncer m_isMt2Debouncer =
        new Debouncer(kLockedDebounceSeconds, Debouncer.DebounceType.kFalling);
    private final Debouncer m_targetListDebouncer =
        new Debouncer(kLockedDebounceSeconds, Debouncer.DebounceType.kFalling);
    private String m_lastNonEmptyTargetList = "";

    /**
     * Constructs a BasicInfoDashboard.
     *
     * @param configInterface   Bot configuration (vision defaults, etc.)
     * @param drivetrain        The swerve drivetrain to get info from
     * @param glassField        Field2d for Glass visualization
     * @param driveAccuracyTester Accuracy test workflow manager
     * @param testSubsystemsCommand Command to test all subsystems
     * @param simCycleResetCmd  Sim-only command to cycle the robot reset position, or null on real robot
     * @param cameraNames       Names of all Limelight cameras to monitor
     */
    public BasicInfoDashboard(
        BotConfigInterface configInterface,
        SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain,
        Field2d glassField,
        DriveAccuracyTester driveAccuracyTester,
        Command testSubsystemsCommand,
        Command simCycleResetCmd,
        List<String> cameraNames) {

        m_drivetrain = drivetrain;
        m_pigeon = drivetrain.getPigeon2();
        m_configInterface = configInterface;
        m_visionEnabled = m_basicInfoTable.getBooleanTopic("VisionEnabled").getEntry(m_configInterface.isVisionEnabledDefault());

        NetworkTable cameraTable = m_basicInfoTable.getSubTable("Camera");
        m_cameraMonitors = new java.util.ArrayList<>();
        for (int i = 0; i < cameraNames.size(); i++) {
            m_cameraMonitors.add(new CameraMonitor(
                cameraNames.get(i),
                "Camera" + (i + 1),
                cameraTable));
        }

        // Publish the default value so the toggle appears in Elastic immediately.
        m_visionEnabled.set(m_configInterface.isVisionEnabledDefault());

        // Init NetworkTables for some dashboard itemms
        SmartDashboard.putString("MAC Address Name", RobotIdentity.getBotName());
        SmartDashboard.putString("Robot Config", configInterface.getConfigName());
        SmartDashboard.putData("GlassField", glassField);
        SmartDashboard.putData("Accuracy Drive Test", driveAccuracyTester.createTapeDropAutoCommand());
        SmartDashboard.putData("Test Subsystems", testSubsystemsCommand);
        if (simCycleResetCmd != null) {
            SmartDashboard.putData("Sim/CycleResetPosition", simCycleResetCmd);
        }
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
        return m_visionEnabled.get(m_configInterface.isVisionEnabledDefault());
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

    /** Updates the vision confidence score for dashboard display. */
    public void updateVisionConfidence(double score) {
        m_cachedVisionConfidence = score;
    }

    /** Updates the single-tag target lock state for dashboard display. */
    public void updateTargetLock(boolean hasLock) {
        m_cachedHasTargetLock = hasLock;
    }

    /** Updates the multi-tag lock state for dashboard display. */
    public void updateMultiTagLock(boolean hasMultiLock) {
        m_cachedHasMultiTagLock = hasMultiLock;
    }

    /** Updates the MegaTag2 active state for dashboard display. */
    public void updateIsLatestMt2(boolean isMt2) {
        m_cachedIsLatestMt2 = isMt2;
    }

    /** Updates the primary tag horizontal offset (tx) for dashboard display. */
    public void updatePrimaryTagTx(double tx) {
        m_cachedTx = tx;
    }

    /** Updates the list of visible tag IDs for dashboard display. */
    public void updateVisibleTagIds(List<Integer> ids) {
        m_cachedTargetList = ids;
    }

    /** Updates the vision error at image-capture time for dashboard display. */
    public void updateVisionErrorAtSnapTime(OptionalDouble error) {
        m_cachedVisionError = error;
    }

    /**
     * Updates the robot motionless state for dashboard display.
     *
     * @param isMotionless true when the robot is stationary
     * @param secondsStill seconds the robot has been continuously still
     */
    public void updateMotionlessState(boolean isMotionless, double secondsStill) {
        m_cachedIsMotionless = isMotionless;
        m_cachedSecondsStill = secondsStill;
    }

    /**
     * Sets the supplier for the vision Kalman filter.
     *
     * @param supplier A Supplier returning the VisionKalmanFilter instance
     */
    // $TODO2 - This seems wrong
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
        }
        else {
            return "EAST";
        }
    }

    /** Called after each Robot Periodic. */
    public void update() {
        /* Gyro and robot heading */
        m_driveHeadingDegrees.set(m_drivetrain.getState().Pose.getRotation().getDegrees());
        m_gyroYawDegrees.set(m_pigeon.getYaw().getValueAsDouble());

        /* Publish other basic info */
        m_allianceWithBlueFallback.set(isRedAlliance() ? "Red" : "Blue");
        m_allianceNt.set(DriverStation.getAlliance().map(Object::toString).orElse("UNKNOWN"));
        m_isDsAttached.set(DriverStation.isDSAttached());
        m_isFmsAttached.set(DriverStation.isFMSAttached());

        /* Publish operator forward direction
            If the operator is in the Blue Alliance Station, this should be 0 degrees (EAST).
            If the operator is in the Red Alliance Station, this should be 180 degrees (WEST).
        */
        double forwardDegrees = m_drivetrain.getOperatorForwardDirection().getDegrees();
        String screenDirection = (Math.abs(forwardDegrees) < 90) ? "EAST" : "WEST";
        m_operatorForwardDirectionDegrees.set(screenDirection);

        m_debugCalculatedForwardDirection.set(getDebugCalculatedForwardString());

        /* Publish vision state */
        m_visionConfidence.set(m_cachedVisionConfidence);
        m_oneLocked.set(m_oneLockedDebouncer.calculate(m_cachedHasTargetLock));
        m_multiLocked.set(m_multiLockedDebouncer.calculate(m_cachedHasMultiTagLock));
        m_isMt2.set(m_isMt2Debouncer.calculate(m_cachedIsLatestMt2));
        m_visionTx.set(m_cachedTx);

        double visErrorMeters = m_cachedVisionError.isPresent() ? m_cachedVisionError.getAsDouble() : 0.0;
        m_visErrorCentimeters.set(100.0 * visErrorMeters);
        m_visErrorMultiTagCentimeters.set(m_cachedHasMultiTagLock ? 100.0 * visErrorMeters : 0.0);

        List<Integer> targets = m_cachedTargetList;
        boolean hasTargets = !targets.isEmpty();
        if (hasTargets) {
            m_lastNonEmptyTargetList = targetListToString(targets);
        }
        // Debounce the "has targets" state - when it goes false, delay before showing empty
        boolean showTargets = m_targetListDebouncer.calculate(hasTargets);
        m_targetList.set(showTargets ? m_lastNonEmptyTargetList : "");

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
            }
            else {
                m_visionKalmanPose.set(new double[] {0, 0, 0});
                m_visionKalmanConverged.set(false);
            }
        }

        /* Publish robot motionless state for Kalman filter */
        m_visionKalmanIsRobotStill.set(m_cachedIsMotionless);
        m_visionKalmanSecondsStill.set(String.format("%.1f", m_cachedSecondsStill));

        /* Update and publish per-camera heartbeat status (only write NT when state may have changed) */
        for (CameraMonitor mon : m_cameraMonitors) {
            if (mon.heartbeat.update()) {
                mon.status.set(mon.heartbeat.getCameraStatus());
            }
        }
    }

}
