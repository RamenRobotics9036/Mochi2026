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

/** Basic logging to dashboard. */
@SuppressWarnings("LineLength")
public class BasicInfoDashboard {
    private final Pigeon2 m_pigeon;
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_drivetrain;
    private boolean m_lastPeriodicWasEnabled = false;
    private String m_lastAllianceString = "Robot disabled";

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


    /**
     * Constructs a BasicInfoDashboard.
     *
     * @param drivetrain The swerve drivetrain to get info from
     */
    public BasicInfoDashboard(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain) {
        m_drivetrain = drivetrain;
        m_pigeon = drivetrain.getPigeon2();
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    private String getAllianceWhenEnabled() {
        double forwardDegrees = m_drivetrain.getOperatorForwardDirection().getDegrees();
        return String.format("%.1f", forwardDegrees);
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

        // Update whether last periodic was enabled
        m_lastPeriodicWasEnabled = DriverStation.isEnabled();
    }
}
