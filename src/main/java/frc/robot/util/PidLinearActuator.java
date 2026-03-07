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
    // FIX #3: guard against atSetpoint() lying after reset()
    private boolean m_pidWasReset;

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
        // FIX #1: baseline must always reflect ACTUAL current position, not target
        m_baselinePositionMM = initialPositionMM;
        m_lastMeasuredPositionMM = initialPositionMM;
        m_lastUpdateTimestampSec = nowSec;
        m_lastMotionTimestampSec = nowSec;
        m_jogActive = false;
        m_skipNextJogIncrement = false;
        m_stallEventLatched = false;
        m_pidWasReset = true; // FIX #3

        m_motor.stopMotor();
        m_pid.reset();
    }

    public double stepBy(double deltaMM) {
        m_jogActive = false;
        m_skipNextJogIncrement = true;
        // FIX #1: step from ACTUAL current position, not baseline
        // baseline was being corrupted by setTargetInternal setting it to target
        return setTargetInternal(getCurrentPositionMM() + deltaMM);
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

        // FIX #3: clear the reset guard now that calculate() has run once with real error
        m_pidWasReset = false;

        output = MathUtil.clamp(output, -m_maxPidOutput, m_maxPidOutput);

        if (currentPositionMM <= m_minPositionMM && output < 0.0) {
            output = 0.0;
        } else if (currentPositionMM >= m_maxPositionMM && output > 0.0) {
            output = 0.0;
        }

        boolean atTarget = isAtTargetInternal();
        if (atTarget) {
            output = 0.0;
            // FIX #1: update baseline to actual position only when truly at target
            m_baselinePositionMM = currentPositionMM;
        }

        // FIX #2: only reset stall timer when actively moving (output >= threshold AND moving)
        // Removed the else branch that reset the timer on low output — that prevented stall detection
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
            // FIX #2: removed `else { m_lastMotionTimestampSec = currentTimeSec; }`
        }

        // Debug instrumentation — remove once motor movement is confirmed
        double error = m_targetPositionMM - currentPositionMM;
        System.out.printf(
            "[PidActuator] pos=%.2f | target=%.2f | setpoint=%.2f | error=%.2f | output=%.3f | atTarget=%b | pidReset=%b%n",
            currentPositionMM, m_targetPositionMM, m_rateLimitedSetpointMM,
            error, output, atTarget, m_pidWasReset);

        if (atTarget) {
            m_motor.stopMotor();
        } else {
            m_motor.set(output);
        }

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
        m_pidWasReset = true; // FIX #3: guard atSetpoint() after reset
        m_pid.reset();
        m_motor.stopMotor();
    }

    private double setTargetInternal(double requestedPositionMM) {
        double previousTargetMM = m_targetPositionMM;
        double clampedTargetMM = clampToLimits(requestedPositionMM);

        m_targetPositionMM = clampedTargetMM;
        // FIX #1: do NOT set m_baselinePositionMM here — baseline is actual position,
        // not the target. Baseline is updated in update() when atTarget is true,
        // and in holdCurrentPosition(). Setting it to target here caused stepBy()
        // to repeatedly step from the target instead of actual position.
        m_stallEventLatched = false;

        return clampedTargetMM - previousTargetMM;
    }

    private boolean isAtTargetInternal() {
        // FIX #3: never declare atTarget immediately after PID reset — velocity error is 0
        if (m_pidWasReset) {
            return false;
        }
        // FIX: removed m_pid.atSetpoint() — unreliable after reset() and with slow actuators.
        // Use explicit position-only checks against positionToleranceMM.
        return Math.abs(m_targetPositionMM - m_rateLimitedSetpointMM) <= m_positionToleranceMM
            && Math.abs(m_targetPositionMM - getCurrentPositionMM()) <= m_positionToleranceMM;
    }

    private double clampToLimits(double positionMM) {
        return MathUtil.clamp(positionMM, m_minPositionMM, m_maxPositionMM);
    }
}
