package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Intake roller subsystem for the 2026 robot.
 *
 * <p>Positive output corresponds to intake direction (counter-clockwise).
 */
public class IntakeRollerSubsystem extends SubsystemBase {

    /** Roller operating modes used by commands. */
    public enum RollerMode {
        INTAKE,
        OUTTAKE,
        EJECT_JAM,
        OFF
    }

    private final SparkMax m_rollerMotor;
    private final RelativeEncoder m_encoder;
    private final Debouncer m_pieceDebouncer;
    private final Debouncer m_jamDebouncer;
    private final Debouncer m_faultDebouncer;

    private RollerMode m_requestedMode = RollerMode.OFF;
    private RollerMode m_appliedMode = RollerMode.OFF;
    private boolean m_pieceDetected = false;
    private boolean m_jamDetected = false;
    private boolean m_faulted = false;
    private boolean m_autoStopEnabled = true;
    private double m_commandedOutput = 0.0;

    // Elastic telemetry publishers
    private final DoublePublisher m_currentPub;
    private final DoublePublisher m_positionPub;
    private final DoublePublisher m_commandedOutputPub;
    private final StringPublisher m_requestedModePub;
    private final StringPublisher m_appliedModePub;
    private final BooleanPublisher m_pieceDetectedPub;
    private final BooleanPublisher m_jamDetectedPub;
    private final BooleanPublisher m_faultedPub;

    public IntakeRollerSubsystem() {
        m_rollerMotor = new SparkMax(IntakeConstants.Roller.kRollerMotorId, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake)
              .inverted(IntakeConstants.Roller.kMotorInverted)
              .smartCurrentLimit(IntakeConstants.Roller.kSmartCurrentLimitAmps);
        m_rollerMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        m_encoder = m_rollerMotor.getEncoder();
        m_pieceDebouncer = new Debouncer(IntakeConstants.Roller.kPieceDetectTimeSeconds, DebounceType.kRising);
        m_jamDebouncer = new Debouncer(IntakeConstants.Roller.kJamDetectTimeSeconds, DebounceType.kRising);
        m_faultDebouncer = new Debouncer(IntakeConstants.Roller.kJamDetectTimeSeconds, DebounceType.kRising);

        NetworkTable table = NetworkTableInstance.getDefault()
            .getTable("Elastic")
            .getSubTable("IntakeRoller");
        m_currentPub = table.getDoubleTopic("CurrentAmps").publish();
        m_positionPub = table.getDoubleTopic("PositionRotations").publish();
        m_commandedOutputPub = table.getDoubleTopic("CommandedOutput").publish();
        m_requestedModePub = table.getStringTopic("RequestedMode").publish();
        m_appliedModePub = table.getStringTopic("AppliedMode").publish();
        m_pieceDetectedPub = table.getBooleanTopic("PieceDetected").publish();
        m_jamDetectedPub = table.getBooleanTopic("JamDetected").publish();
        m_faultedPub = table.getBooleanTopic("Faulted").publish();
    }

    /** Sets the desired roller mode. */
    public void setMode(RollerMode mode) {
        if (m_faulted) {
            return;
        }
        m_requestedMode = mode;
    }

    /** Immediately stops the roller motor. */
    public void stop() {
        m_requestedMode = RollerMode.OFF;
        m_rollerMotor.stopMotor();
    }

    public void setAutoStopEnabled(boolean enabled) {
        m_autoStopEnabled = enabled;
    }

    public RollerMode getRequestedMode() {
        return m_requestedMode;
    }

    public RollerMode getAppliedMode() {
        return m_appliedMode;
    }

    public boolean isPieceDetected() {
        return m_pieceDetected;
    }

    public boolean isJamDetected() {
        return m_jamDetected;
    }

    public boolean isFaulted() {
        return m_faulted;
    }

    public void clearFaults() {
        m_faulted = false;
    }

    public double getCurrentAmps() {
        return m_rollerMotor.getOutputCurrent();
    }

    public double getPositionRotations() {
        return m_encoder.getPosition();
    }

    @Override
    public void periodic() {
        updateDetections();
        updateFaults();
        applyOutput();
        publishTelemetry();
    }

    private void updateDetections() {
        boolean intakeCommanded = m_requestedMode == RollerMode.INTAKE;
        boolean outputActive = Math.abs(m_rollerMotor.getAppliedOutput())
            >= IntakeConstants.Roller.kMinOutputForDetection;
        double current = getCurrentAmps();

        m_pieceDetected = m_pieceDebouncer.calculate(
            intakeCommanded
                && outputActive
                && current >= IntakeConstants.Roller.kPieceDetectCurrentThresholdAmps
        );

        m_jamDetected = m_jamDebouncer.calculate(
            intakeCommanded
                && outputActive
                && current >= IntakeConstants.Roller.kJamCurrentThresholdAmps
        );
    }

    private void updateFaults() {
        if (m_faulted) {
            return;
        }

        boolean outputActive = Math.abs(m_rollerMotor.getAppliedOutput())
            >= IntakeConstants.Roller.kMinOutputForDetection;
        boolean faultCondition = outputActive
            && m_requestedMode == RollerMode.OFF
            && getCurrentAmps() >= IntakeConstants.Roller.kJamCurrentThresholdAmps;

        if (m_faultDebouncer.calculate(faultCondition)) {
            m_faulted = true;
            stop();
        }
    }

    private void applyOutput() {
        RollerMode modeToApply = m_requestedMode;
        if (m_autoStopEnabled && m_requestedMode == RollerMode.INTAKE && m_pieceDetected) {
            modeToApply = RollerMode.OFF;
        }
        if (m_faulted) {
            modeToApply = RollerMode.OFF;
        }

        double output = switch (modeToApply) {
            case INTAKE -> IntakeConstants.Roller.kIntakeSpeed;
            case OUTTAKE -> -IntakeConstants.Roller.kOuttakeSpeed;
            case EJECT_JAM -> -IntakeConstants.Roller.kEjectJamSpeed;
            case OFF -> 0.0;
        };

        m_commandedOutput = output;
        m_appliedMode = modeToApply;
        m_rollerMotor.set(output);
    }

    private void publishTelemetry() {
        m_currentPub.set(getCurrentAmps());
        m_positionPub.set(getPositionRotations());
        m_commandedOutputPub.set(m_commandedOutput);
        m_requestedModePub.set(m_requestedMode.name());
        m_appliedModePub.set(m_appliedMode.name());
        m_pieceDetectedPub.set(m_pieceDetected);
        m_jamDetectedPub.set(m_jamDetected);
        m_faultedPub.set(m_faulted);
    }
}
