package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/** PID-controlled linear actuator wrapper with analog potentiometer feedback. */
public class PidLinearActuator {
    private final PWMSparkMax m_motor;
    private final AnalogPotentiometer m_potentiometer;
    private final PIDController m_pid;

    private final double m_minPositionMM;
    private final double m_maxPositionMM;
    private final double m_maxRateMMPerSec;
    private final double m_positionToleranceMM;
    private final double m_maxPidOutput;
    private final double m_minMovementMM;
    private final double m_stallTimeoutSec;
    private final double m_stallOutputThreshold;

    private double m_targetPositionMM;
    private double m_rateLimitedSetpointMM;
    private double m_baselinePositionMM;
    private double m_lastMeasuredPositionMM;
    private double m_lastUpdateTimestampSec;
    private double m_lastMotionTimestampSec;
    private boolean m_jogActive;
    private boolean m_skipNextJogIncrement;
    private boolean m_stallEventLatched;

    // Add: track whether at least one full update cycle has completed
    private boolean m_hasUpdatedOnce;

    public PidLinearActuator(
            int pwmChannel,
            int analogChannel,
            boolean motorInverted,
            double potRangeMM,
            double potOffsetMM,
            double minPositionMM,
            double maxPositionMM,
            double maxRateMMPerSec,
            double loopPeriodSec,
            double kP,
            double kI,
            double kD,
            double positionToleranceMM,
            double velocityToleranceMMPerSec,
            double maxPidOutput,
            double minMovementMM,
            double stallTimeoutSec,
            double stallOutputThreshold) {
        if (potRangeMM == 0.0) {
            throw new IllegalArgumentException("Potentiometer range must be non-zero.");
        }
        if (maxPositionMM < minPositionMM) {
            throw new IllegalArgumentException("Max position must be >= min position.");
        }
        if (maxRateMMPerSec <= 0.0) {
            throw new IllegalArgumentException("Max rate must be positive.");
        }
        if (loopPeriodSec <= 0.0) {
            throw new IllegalArgumentException("Loop period must be positive.");
        }
        if (positionToleranceMM < 0.0) {
            throw new IllegalArgumentException("Position tolerance cannot be negative.");
        }
        if (velocityToleranceMMPerSec < 0.0) {
            throw new IllegalArgumentException("Velocity tolerance cannot be negative.");
        }
        if (maxPidOutput <= 0.0 || maxPidOutput > 1.0) {
            throw new IllegalArgumentException("Max PID output must be in (0, 1].");
        }
        if (minMovementMM < 0.0) {
            throw new IllegalArgumentException("Minimum movement cannot be negative.");
        }
        if (stallTimeoutSec <= 0.0) {
            throw new IllegalArgumentException("Stall timeout must be positive.");
        }
        if (stallOutputThreshold < 0.0 || stallOutputThreshold > 1.0) {
            throw new IllegalArgumentException("Stall output threshold must be in [0, 1].");
        }

        m_motor = new PWMSparkMax(pwmChannel);
        m_motor.setInverted(motorInverted);

        m_potentiometer = new AnalogPotentiometer(analogChannel, potRangeMM, potOffsetMM);
        m_pid = new PIDController(kP, kI, kD, loopPeriodSec);
        m_pid.setTolerance(positionToleranceMM, velocityToleranceMMPerSec);

        m_minPositionMM = minPositionMM;
        m_maxPositionMM = maxPositionMM;
        m_maxRateMMPerSec = maxRateMMPerSec;
        m_positionToleranceMM = positionToleranceMM;
        m_maxPidOutput = maxPidOutput;
        m_minMovementMM = minMovementMM;
        m_stallTimeoutSec = stallTimeoutSec;
        m_stallOutputThreshold = stallOutputThreshold;

        double initialPositionMM = getCurrentPositionMM();
        double nowSec = Timer.getFPGATimestamp();

        m_targetPositionMM = initialPositionMM;
        m_rateLimitedSetpointMM = initialPositionMM;
        m_baselinePositionMM = initialPositionMM;
        m_lastMeasuredPositionMM = initialPositionMM;
        m_lastUpdateTimestampSec = nowSec;
        m_lastMotionTimestampSec = nowSec;
        m_jogActive = false;
        m_skipNextJogIncrement = false;
        m_stallEventLatched = false;

        // Initialize the new flag
        m_hasUpdatedOnce = false;

        m_motor.stopMotor();
        m_pid.reset();
    }

    public double stepBy(double deltaMM) {
        m_jogActive = false;
        m_skipNextJogIncrement = true;
        return setTargetInternal(m_baselinePositionMM + deltaMM);
    }

    public double setTargetPositionMM(double positionMM) {
        m_jogActive = false;
        m_skipNextJogIncrement = false;
        return setTargetInternal(positionMM);
    }

    public double jogBy(double deltaMM) {
        if (deltaMM == 0.0) {
            return 0.0;
        }

        if (m_skipNextJogIncrement) {
            m_skipNextJogIncrement = false;
            return Double.NaN;
        }

        m_jogActive = true;
        return setTargetInternal(m_targetPositionMM + deltaMM);
    }

    public boolean stopJog() {
        m_skipNextJogIncrement = false;

        if (!m_jogActive) {
            return false;
        }

        m_jogActive = false;
        holdCurrentPosition();
        return true;
    }

    public void update() {
        double currentTimeSec = Timer.getFPGATimestamp();
        double dtSeconds = Math.max(0.0, currentTimeSec - m_lastUpdateTimestampSec);
        double currentPositionMM = getCurrentPositionMM();
        double movementThisCycleMM = Math.abs(currentPositionMM - m_lastMeasuredPositionMM);

        advanceRateLimitedSetpoint(dtSeconds);

        double output = m_pid.calculate(currentPositionMM, m_rateLimitedSetpointMM);
        output = MathUtil.clamp(output, -m_maxPidOutput, m_maxPidOutput);

        // Software limit stops
        if (currentPositionMM <= m_minPositionMM && output < 0.0) {
            output = 0.0;
        } else if (currentPositionMM >= m_maxPositionMM && output > 0.0) {
            output = 0.0;
        }

        // Guard: never declare atTarget on the very first cycle — PID internal
        // velocity error is zero by default which makes atSetpoint() lie.
        boolean atTarget = m_hasUpdatedOnce && isAtTargetInternal();
        m_hasUpdatedOnce = true;

        if (atTarget) {
            output = 0.0;
            m_baselinePositionMM = currentPositionMM;
        }

        if (!atTarget && Math.abs(output) >= m_stallOutputThreshold) {
            if (movementThisCycleMM >= m_minMovementMM) {
                m_lastMotionTimestampSec = currentTimeSec;
            } else if (currentTimeSec - m_lastMotionTimestampSec >= m_stallTimeoutSec) {
                holdCurrentPosition();
                m_stallEventLatched = true;
                currentPositionMM = m_baselinePositionMM;
                output = 0.0;
                atTarget = true;
            }
        } else if (!atTarget) {
            // Only reset stall timer when output is low AND not yet at target,
            // so a brief output dip doesn't prevent stall detection.
            // Do NOT reset timer here — remove the else branch that did so.
        }
        // NOTE: removed the old `else { m_lastMotionTimestampSec = currentTimeSec; }`
        // That was resetting the stall timer whenever output was below threshold,
        // which prevented stall detection from ever firing.

        if (atTarget) {
            m_motor.stopMotor();
        } else {
            m_motor.set(output);
        }

        // Debug instrumentation — remove after confirmed working
        System.out.printf(
            "[PidActuator] pos=%.2f mm | setpoint=%.2f mm | target=%.2f mm | output=%.3f | atTarget=%b%n",
            currentPositionMM, m_rateLimitedSetpointMM, m_targetPositionMM, output, atTarget);

        m_lastMeasuredPositionMM = currentPositionMM;
        m_lastUpdateTimestampSec = currentTimeSec;
    }

    public double getCurrentPositionMM() {
        return clampToLimits(m_potentiometer.get());
    }

    public double getTargetPositionMM() {
        return m_targetPositionMM;
    }

    public double getBaselinePositionMM() {
        return m_baselinePositionMM;
    }

    public boolean isAtTarget() {
        return isAtTargetInternal();
    }

    public boolean consumeStallEvent() {
        boolean hadStallEvent = m_stallEventLatched;
        m_stallEventLatched = false;
        return hadStallEvent;
    }

    private void advanceRateLimitedSetpoint(double dtSeconds) {
        if (dtSeconds <= 0.0) {
            return;
        }

        double maxStepMM = m_maxRateMMPerSec * dtSeconds;
        double setpointErrorMM = m_targetPositionMM - m_rateLimitedSetpointMM;

        if (Math.abs(setpointErrorMM) <= maxStepMM) {
            m_rateLimitedSetpointMM = m_targetPositionMM;
            return;
        }

        m_rateLimitedSetpointMM += Math.copySign(maxStepMM, setpointErrorMM);
    }

    private void holdCurrentPosition() {
        double currentPositionMM = getCurrentPositionMM();

        m_targetPositionMM = currentPositionMM;
        m_rateLimitedSetpointMM = currentPositionMM;
        m_baselinePositionMM = currentPositionMM;
        m_lastMotionTimestampSec = Timer.getFPGATimestamp();
        m_hasUpdatedOnce = false; // Reset so next update doesn't fire atTarget immediately
        m_pid.reset();
        m_motor.stopMotor();
    }

    private double setTargetInternal(double requestedPositionMM) {
        double previousTargetMM = m_targetPositionMM;
        double clampedTargetMM = clampToLimits(requestedPositionMM);

        m_targetPositionMM = clampedTargetMM;
        m_baselinePositionMM = clampedTargetMM;
        m_stallEventLatched = false;

        return clampedTargetMM - previousTargetMM;
    }

    private boolean isAtTargetInternal() {
        // Do NOT use m_pid.atSetpoint() — its velocity error is 0 on first cycle
        // and the velocity tolerance interacts poorly with slow actuators.
        // Use explicit position checks only.
        double rateLimitedError = Math.abs(m_targetPositionMM - m_rateLimitedSetpointMM);
        double positionError = Math.abs(m_targetPositionMM - getCurrentPositionMM());
        return rateLimitedError <= m_positionToleranceMM
            && positionError <= m_positionToleranceMM;
    }

    private double clampToLimits(double positionMM) {
        return MathUtil.clamp(positionMM, m_minPositionMM, m_maxPositionMM);
    }
}
