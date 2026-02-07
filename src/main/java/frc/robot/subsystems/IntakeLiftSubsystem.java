package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
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
 * Intake lift subsystem for the 2026 robot.
 *
 * <p>Assumptions:
 * Encoder units are rotations and must be converted to arm angle (radians) using
 * {@link IntakeConstants.Lift#kRotationsToRadians}. Limit switches are wired
 * normally-closed at the SPARK FLEX controller inputs.
 */
public class IntakeLiftSubsystem extends SubsystemBase {

    /** Named lift positions for command intent. */
    public enum LiftPosition {
        STOWED(IntakeConstants.Lift.kStowedPositionRotations),
        DEPLOYED(IntakeConstants.Lift.kDeployedPositionRotations);

        private final double m_rotations;

        LiftPosition(double rotations) {
            m_rotations = rotations;
        }

        public double getRotations() {
            return m_rotations;
        }
    }

    private final SparkFlex m_leftMotor;
    private final SparkFlex m_rightMotor;
    private final SparkClosedLoopController m_leftController;
    private final SparkClosedLoopController m_rightController;
    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;
    private final SparkLimitSwitch m_leftForwardLimit;
    private final SparkLimitSwitch m_leftReverseLimit;
    private final SparkLimitSwitch m_rightForwardLimit;
    private final SparkLimitSwitch m_rightReverseLimit;
    private final ArmFeedforward m_feedforward;
    private final Debouncer m_leftStallDebouncer;
    private final Debouncer m_rightStallDebouncer;

    private double m_targetRotations;
    private LiftPosition m_targetPosition = LiftPosition.STOWED;
    private boolean m_hasTarget = false;
    private boolean m_faulted = false;
    private String m_faultReason = "None";

    // Elastic telemetry publishers
    private final DoublePublisher m_leftCurrentPub;
    private final DoublePublisher m_rightCurrentPub;
    private final DoublePublisher m_leftPositionPub;
    private final DoublePublisher m_rightPositionPub;
    private final DoublePublisher m_targetPositionPub;
    private final StringPublisher m_targetNamePub;
    private final BooleanPublisher m_faultedPub;
    private final StringPublisher m_faultReasonPub;
    private final BooleanPublisher m_leftForwardLimitPub;
    private final BooleanPublisher m_leftReverseLimitPub;
    private final BooleanPublisher m_rightForwardLimitPub;
    private final BooleanPublisher m_rightReverseLimitPub;

    public IntakeLiftSubsystem() {
        m_leftMotor = new SparkFlex(IntakeConstants.Lift.kLeftMotorId, SparkLowLevel.MotorType.kBrushless);
        m_rightMotor = new SparkFlex(IntakeConstants.Lift.kRightMotorId, SparkLowLevel.MotorType.kBrushless);

        SparkFlexConfig leftConfig = buildConfig(IntakeConstants.Lift.kLeftMotorInverted);
        SparkFlexConfig rightConfig = buildConfig(IntakeConstants.Lift.kRightMotorInverted);

        m_leftMotor.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        m_rightMotor.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        m_leftController = m_leftMotor.getClosedLoopController();
        m_rightController = m_rightMotor.getClosedLoopController();
        m_leftEncoder = m_leftMotor.getEncoder();
        m_rightEncoder = m_rightMotor.getEncoder();

        m_leftForwardLimit = m_leftMotor.getForwardLimitSwitch();
        m_leftReverseLimit = m_leftMotor.getReverseLimitSwitch();
        m_rightForwardLimit = m_rightMotor.getForwardLimitSwitch();
        m_rightReverseLimit = m_rightMotor.getReverseLimitSwitch();

        m_feedforward = new ArmFeedforward(
            IntakeConstants.Lift.kS,
            IntakeConstants.Lift.kG,
            IntakeConstants.Lift.kV,
            IntakeConstants.Lift.kA
        );

        m_leftStallDebouncer = new Debouncer(IntakeConstants.Lift.kStallTimeSeconds, DebounceType.kRising);
        m_rightStallDebouncer = new Debouncer(IntakeConstants.Lift.kStallTimeSeconds, DebounceType.kRising);

        // Initialize target to current position to prevent unexpected motion on boot.
        m_targetRotations = getPositionRotations();
        m_hasTarget = true;

        NetworkTable table = NetworkTableInstance.getDefault()
            .getTable("Elastic")
            .getSubTable("IntakeLift");
        m_leftCurrentPub = table.getDoubleTopic("LeftCurrentAmps").publish();
        m_rightCurrentPub = table.getDoubleTopic("RightCurrentAmps").publish();
        m_leftPositionPub = table.getDoubleTopic("LeftPositionRotations").publish();
        m_rightPositionPub = table.getDoubleTopic("RightPositionRotations").publish();
        m_targetPositionPub = table.getDoubleTopic("TargetPositionRotations").publish();
        m_targetNamePub = table.getStringTopic("TargetPositionName").publish();
        m_faultedPub = table.getBooleanTopic("Faulted").publish();
        m_faultReasonPub = table.getStringTopic("FaultReason").publish();
        m_leftForwardLimitPub = table.getBooleanTopic("LeftForwardLimitPressed").publish();
        m_leftReverseLimitPub = table.getBooleanTopic("LeftReverseLimitPressed").publish();
        m_rightForwardLimitPub = table.getBooleanTopic("RightForwardLimitPressed").publish();
        m_rightReverseLimitPub = table.getBooleanTopic("RightReverseLimitPressed").publish();
    }

    private SparkFlexConfig buildConfig(boolean inverted) {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake)
              .inverted(inverted)
              .smartCurrentLimit(IntakeConstants.Lift.kSmartCurrentLimitAmps);

        ClosedLoopConfig closedLoop = config.closedLoop;
        closedLoop.p(IntakeConstants.Lift.kP)
                  .i(IntakeConstants.Lift.kI)
                  .d(IntakeConstants.Lift.kD)
                  .outputRange(IntakeConstants.Lift.kMinOutput, IntakeConstants.Lift.kMaxOutput);

        SoftLimitConfig softLimit = config.softLimit;
        softLimit.forwardSoftLimit(IntakeConstants.Lift.kSoftLimitForwardRotations);
        softLimit.reverseSoftLimit(IntakeConstants.Lift.kSoftLimitReverseRotations);
        softLimit.forwardSoftLimitEnabled(true);
        softLimit.reverseSoftLimitEnabled(true);

        LimitSwitchConfig limitSwitch = config.limitSwitch;
        limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
        limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
        limitSwitch.forwardLimitSwitchTriggerBehavior(
            IntakeConstants.Lift.kEnableForwardLimitSwitch
                ? LimitSwitchConfig.Behavior.kStopMovingMotor
                : LimitSwitchConfig.Behavior.kKeepMovingMotor
        );
        limitSwitch.reverseLimitSwitchTriggerBehavior(
            IntakeConstants.Lift.kEnableReverseLimitSwitch
                ? LimitSwitchConfig.Behavior.kStopMovingMotor
                : LimitSwitchConfig.Behavior.kKeepMovingMotor
        );

        return config;
    }

    /** Command the lift to a named position. */
    public void setTargetPosition(LiftPosition position) {
        setTargetRotations(position.getRotations());
        m_targetPosition = position;
    }

    /** Command the lift to a raw position in rotations (clamped to soft limits). */
    public void setTargetRotations(double rotations) {
        if (m_faulted) {
            return;
        }
        m_targetRotations = MathUtil.clamp(
            rotations,
            IntakeConstants.Lift.kSoftLimitReverseRotations,
            IntakeConstants.Lift.kSoftLimitForwardRotations
        );
        m_hasTarget = true;
    }

    /** Stops the lift motors immediately. */
    public void stop() {
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
        m_hasTarget = false;
    }

    /** Clears latched faults after a manual inspection. */
    public void clearFaults() {
        m_faulted = false;
        m_faultReason = "None";
    }

    public boolean isFaulted() {
        return m_faulted;
    }

    public double getLeftPositionRotations() {
        return m_leftEncoder.getPosition();
    }

    public double getRightPositionRotations() {
        return m_rightEncoder.getPosition();
    }

    public double getPositionRotations() {
        return (getLeftPositionRotations() + getRightPositionRotations()) / 2.0;
    }

    public double getTargetRotations() {
        return m_targetRotations;
    }

    public LiftPosition getTargetPosition() {
        return m_targetPosition;
    }

    public boolean atTarget() {
        return Math.abs(getPositionRotations() - m_targetRotations)
            <= IntakeConstants.Lift.kPositionToleranceRotations;
    }

    @Override
    public void periodic() {
        updateFaults();
        applyClosedLoop();
        publishTelemetry();
    }

    private void applyClosedLoop() {
        if (!m_hasTarget || m_faulted) {
            return;
        }

        double angleRadians =
            (getPositionRotations() * IntakeConstants.Lift.kRotationsToRadians)
                + IntakeConstants.Lift.kAngleOffsetRadians;

        double feedforwardVolts = m_feedforward.calculate(angleRadians, 0.0);

        m_leftController.setSetpoint(
            m_targetRotations,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforwardVolts,
            ArbFFUnits.kVoltage
        );
        m_rightController.setSetpoint(
            m_targetRotations,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforwardVolts,
            ArbFFUnits.kVoltage
        );
    }

    private void updateFaults() {
        if (m_faulted) {
            return;
        }

        boolean leftStall = m_leftStallDebouncer.calculate(
            Math.abs(m_leftMotor.getOutputCurrent()) >= IntakeConstants.Lift.kStallCurrentThresholdAmps
                && Math.abs(m_leftMotor.getAppliedOutput()) >= IntakeConstants.Lift.kStallMinAppliedOutput
        );
        boolean rightStall = m_rightStallDebouncer.calculate(
            Math.abs(m_rightMotor.getOutputCurrent()) >= IntakeConstants.Lift.kStallCurrentThresholdAmps
                && Math.abs(m_rightMotor.getAppliedOutput()) >= IntakeConstants.Lift.kStallMinAppliedOutput
        );

        if (leftStall || rightStall) {
            m_faulted = true;
            m_faultReason = "Lift stall detected";
            stop();
        }
    }

    private void publishTelemetry() {
        m_leftCurrentPub.set(m_leftMotor.getOutputCurrent());
        m_rightCurrentPub.set(m_rightMotor.getOutputCurrent());
        m_leftPositionPub.set(getLeftPositionRotations());
        m_rightPositionPub.set(getRightPositionRotations());
        m_targetPositionPub.set(m_targetRotations);
        m_targetNamePub.set(m_targetPosition.name());
        m_faultedPub.set(m_faulted);
        m_faultReasonPub.set(m_faultReason);
        m_leftForwardLimitPub.set(m_leftForwardLimit.isPressed());
        m_leftReverseLimitPub.set(m_leftReverseLimit.isPressed());
        m_rightForwardLimitPub.set(m_rightForwardLimit.isPressed());
        m_rightReverseLimitPub.set(m_rightReverseLimit.isPressed());
    }
}
