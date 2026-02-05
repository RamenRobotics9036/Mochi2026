package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

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

    private DoubleSupplier m_visionConfidenceSupplier = null;
    private IntSupplier m_numLockedTagsSupplier = null;
    private DoubleSupplier m_txSupplier = null;

    /**
     * Constructs a BasicInfoDashboard.
     *
     * @param drivetrain The swerve drivetrain to get info from
     */
    public BasicInfoDashboard(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain) {
        m_drivetrain = drivetrain;
        m_pigeon = drivetrain.getPigeon2();
    }

    /**
     * Sets the supplier for vision confidence score.
     *
     * @param supplier A DoubleSupplier returning confidence 0-100
     */
    public void setVisionConfidenceSupplier(DoubleSupplier supplier) {
        m_visionConfidenceSupplier = supplier;
    }

    /**
     * Sets the supplier for number of locked AprilTags.
     *
     * @param supplier An IntSupplier returning the tag count
     */
    public void setNumLockedTagsSupplier(IntSupplier supplier) {
        m_numLockedTagsSupplier = supplier;
    }

    /**
     * Sets the supplier for vision tx (horizontal offset to target).
     *
     * @param supplier A DoubleSupplier returning tx in degrees
     */
    public void setTxSupplier(DoubleSupplier supplier) {
        m_txSupplier = supplier;
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
            m_oneLocked.set(numTags >= 1);
            m_multiLocked.set(numTags >= 2);
        }
        if (m_txSupplier != null) {
            m_visionTx.set(m_txSupplier.getAsDouble());
        }
    }
}
