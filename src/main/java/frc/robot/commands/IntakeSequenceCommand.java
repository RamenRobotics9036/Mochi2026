package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeLiftSubsystem;
import frc.robot.subsystems.IntakeLiftSubsystem.LiftPosition;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem.RollerMode;

/**
 * Intake sequence command that coordinates lift and roller subsystems.
 *
 * <p>State machine: STOWED → DEPLOY → INTAKE → STOW.
 */
public class IntakeSequenceCommand extends Command {

    /** Intake sequence states. */
    public enum IntakeState {
        STOWED,
        DEPLOYING,
        INTAKING,
        CLEAR_JAM,
        STOWING,
        DONE,
        FAULT
    }

    private final IntakeLiftSubsystem m_lift;
    private final IntakeRollerSubsystem m_roller;
    private final Timer m_stateTimer = new Timer();
    private final Timer m_jamTimer = new Timer();

    private IntakeState m_state = IntakeState.STOWED;
    private String m_lastTransitionReason = "None";

    // Elastic telemetry publishers
    private final StringPublisher m_statePub;
    private final StringPublisher m_reasonPub;
    private final DoublePublisher m_stateTimerPub;

    public IntakeSequenceCommand(IntakeLiftSubsystem lift, IntakeRollerSubsystem roller) {
        m_lift = lift;
        m_roller = roller;
        addRequirements(m_lift, m_roller);

        NetworkTable table = NetworkTableInstance.getDefault()
            .getTable("Elastic")
            .getSubTable("IntakeSequence");
        m_statePub = table.getStringTopic("State").publish();
        m_reasonPub = table.getStringTopic("LastTransition").publish();
        m_stateTimerPub = table.getDoubleTopic("StateTimerSeconds").publish();
    }

    @Override
    public void initialize() {
        m_stateTimer.reset();
        m_stateTimer.start();
        m_jamTimer.stop();
        m_jamTimer.reset();
        transitionTo(IntakeState.STOWED, "init");
    }

    @Override
    public void execute() {
        if (m_lift.isFaulted() || m_roller.isFaulted()) {
            transitionTo(IntakeState.FAULT, "subsystem fault");
        }

        switch (m_state) {
            case STOWED -> {
                m_lift.setTargetPosition(LiftPosition.STOWED);
                m_roller.setMode(RollerMode.OFF);
                if (m_lift.atTarget()) {
                    transitionTo(IntakeState.DEPLOYING, "stowed");
                }
            }
            case DEPLOYING -> {
                m_lift.setTargetPosition(LiftPosition.DEPLOYED);
                if (m_lift.atTarget()) {
                    transitionTo(IntakeState.INTAKING, "deployed");
                } else if (m_stateTimer.hasElapsed(IntakeConstants.Sequence.kDeployTimeoutSeconds)) {
                    transitionTo(IntakeState.FAULT, "deploy timeout");
                }
            }
            case INTAKING -> {
                m_roller.setMode(RollerMode.INTAKE);
                if (m_roller.isPieceDetected()) {
                    transitionTo(IntakeState.STOWING, "piece detected");
                } else if (m_roller.isJamDetected()) {
                    transitionTo(IntakeState.CLEAR_JAM, "jam detected");
                }
            }
            case CLEAR_JAM -> {
                m_roller.setMode(RollerMode.EJECT_JAM);
                if (m_jamTimer.hasElapsed(IntakeConstants.Sequence.kJamReverseSeconds)) {
                    transitionTo(IntakeState.INTAKING, "jam cleared");
                }
            }
            case STOWING -> {
                m_roller.setMode(RollerMode.OFF);
                m_lift.setTargetPosition(LiftPosition.STOWED);
                if (m_lift.atTarget()) {
                    transitionTo(IntakeState.DONE, "stowed");
                } else if (m_stateTimer.hasElapsed(IntakeConstants.Sequence.kStowTimeoutSeconds)) {
                    transitionTo(IntakeState.FAULT, "stow timeout");
                }
            }
            case DONE -> {
                m_roller.setMode(RollerMode.OFF);
                m_lift.setTargetPosition(LiftPosition.STOWED);
            }
            case FAULT -> {
                m_roller.stop();
                m_lift.stop();
            }
        }

        publishTelemetry();
    }

    @Override
    public boolean isFinished() {
        return m_state == IntakeState.DONE || m_state == IntakeState.FAULT;
    }

    @Override
    public void end(boolean interrupted) {
        if (m_state != IntakeState.FAULT) {
            m_roller.setMode(RollerMode.OFF);
            m_lift.setTargetPosition(LiftPosition.STOWED);
        } else {
            m_roller.stop();
            m_lift.stop();
        }
    }

    private void transitionTo(IntakeState newState, String reason) {
        if (m_state == newState) {
            return;
        }
        m_state = newState;
        m_lastTransitionReason = reason;
        m_stateTimer.reset();
        m_stateTimer.start();

        if (newState == IntakeState.CLEAR_JAM) {
            m_jamTimer.reset();
            m_jamTimer.start();
        } else {
            m_jamTimer.stop();
        }
    }

    private void publishTelemetry() {
        m_statePub.set(m_state.name());
        m_reasonPub.set(m_lastTransitionReason);
        m_stateTimerPub.set(m_stateTimer.get());
    }
}
