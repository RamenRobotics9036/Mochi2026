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

        if (currentPositionMM <= m_minPositionMM && output < 0.0) {
            output = 0.0;
        } else if (currentPositionMM >= m_maxPositionMM && output > 0.0) {
            output = 0.0;
        }

        boolean atTarget = isAtTargetInternal();
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
        } else {
            m_lastMotionTimestampSec = currentTimeSec;
        }

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
        return Math.abs(m_targetPositionMM - m_rateLimitedSetpointMM) <= m_positionToleranceMM
            && Math.abs(m_targetPositionMM - getCurrentPositionMM()) <= m_positionToleranceMM
            && m_pid.atSetpoint();
    }

    private double clampToLimits(double positionMM) {
        return MathUtil.clamp(positionMM, m_minPositionMM, m_maxPositionMM);
    }
}
