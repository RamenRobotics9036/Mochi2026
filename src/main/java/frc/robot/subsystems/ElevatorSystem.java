package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;

/**
 * Subsystem responsible for the robot's vertical elevator.
 * 
 * <p>Uses a leader-follower SPARK MAX setup to control a carriage via PID position control.
 * Features an automatic zeroing sequence (homing) using a magnetic limit switch at the base.
 */
@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class ElevatorSystem extends SubsystemBase{
    /** The main motor that runs PID and receives position commands. */
    private final SparkMax m_leaderMotor = new SparkMax(ElevatorConstants.kLeaderMotorID, MotorType.kBrushless);
    /** The motor mechanically linked to the leader, configured to follow its output. */
    private final SparkMax m_followMotor = new SparkMax(ElevatorConstants.kFollowMotorID, MotorType.kBrushless);

    // Motor configurations
    private SparkMaxConfig m_leaderConfig = new SparkMaxConfig();
    private SparkMaxConfig m_followConfig = new SparkMaxConfig();
    private RelativeEncoder m_encoder = m_leaderMotor.getEncoder();
    private SparkClosedLoopController m_PIDController = m_leaderMotor.getClosedLoopController();

    /** The target height the elevator is currently attempting to reach/hold. */
    private double m_desiredPosition;
    /** The current height of the elevator carriage based on the relative encoder. */
    private double m_encoderPosition;
    /** True if the physical limit switch at the bottom of the elevator is triggered. */
    private boolean m_limitReached = false;
    /** Magnetic or mechanical limit switch used for homing the elevator on startup. */
    private DigitalInput m_limitSwitch= new DigitalInput(ElevatorConstants.kDIOIndex);
    /* Sensor will reset a relative encoder when the elevator lowers fully
     * Encoder will be used to prevent arm from going too high*/

    /** State machine used to handle initial homing and operational readiness. */
    enum states {
        /** Initial state: Elevator must be lowered until the limit switch is hit to zero the encoder. */
        INIT,
        /** Transition state: Limit switch is triggered and encoder is reset to zero. */
        READYLOW,
        /** Fully operational: Elevator is zeroed and can safely move within soft limits. */
        READY,
    }
    private states m_state = states.INIT;
    
    /**
     * Constructs the ElevatorSystem.
     * Configures motor limits, PID constants, and mechanical gear ratios.
     */
    public ElevatorSystem() {
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig
            .p(4) // Aggressive P gain for holding weight
            .i(0)
            .d(0);
        closedLoopConfig.positionWrappingEnabled(false);
        closedLoopConfig.outputRange(-ElevatorConstants.elevatorMaxSpeed, ElevatorConstants.elevatorMaxSpeed);
        //closedLoopConfig.minOutput();
        //closedLoopConfig.maxOutput();

        EncoderConfig encoderConfig = new EncoderConfig();
        // Converts raw motor rotations into real-world units (e.g., inches or meters)
        encoderConfig.positionConversionFactor(ElevatorConstants.kRotationToElevatorRatio);
        // encoderConfig.velocityConversionFactor(ElevatorConstants.kRotationToElevatorRatio / 60);

        // Configure Leader Motor
        m_leaderConfig.idleMode(SparkBaseConfig.IdleMode.kBrake); // Hold position when power is cut
        m_leaderConfig.smartCurrentLimit(ElevatorConstants.kElevatorStallLimit);
        m_leaderConfig.inverted(true);
        m_leaderConfig.apply(closedLoopConfig);
        m_leaderConfig.apply(encoderConfig);
        m_leaderMotor.configure(m_leaderConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);

        // Configure Follower Motor
        m_followConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_followConfig.smartCurrentLimit(ElevatorConstants.kElevatorStallLimit);
        m_followConfig.inverted(false); // Inverted relative to leader if motors face opposite directions
        m_followConfig.follow(ElevatorConstants.kLeaderMotorID);
        m_followMotor.configure(m_followConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);

        m_encoderPosition = m_encoder.getPosition();

        initShuffleboard();
    }

    /** Sets up telemetry for debugging; disabled in Competition Mode to save CAN bandwidth. */
    private void initShuffleboard(){
        if (!OperatorConstants.kCompetitionMode){
            ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
            tab.addNumber("Desired Position", () ->  m_desiredPosition);
            tab.addNumber("Position", this::getPosition);
            tab.addBoolean("Limit Switch Value", () -> m_limitReached);
            tab.addNumber("Relative Encoder Position", () ->  m_encoderPosition);
            tab.addString("Elevator State", () -> m_state.toString());
        }
    }

    /**
     * Handles sensor updates and state transitions.
     * Continuously monitors the limit switch to ensure encoder accuracy.
     */
    @Override
    public void periodic(){
        // update cached values
        m_limitReached = m_limitSwitch.get();
        m_encoderPosition = m_encoder.getPosition();

        switch (m_state) {
        
            case INIT:
                // Wait for the elevator to hit the bottom for the first time
                if (m_limitReached) {
                    m_state = states.READYLOW;
                    resetEncoder();
                    // initializeMotorConfig is a no-op since we do not use the brake to keep the elevator in place
                    // initializeMotorConfig(false); // Set to false so we can set the elevator down
                }
                break;
            case READYLOW:
                // Once zeroed, wait for it to move off the switch to become fully 'READY'
                if (!m_limitReached) {
                    m_state = states.READY;
                }
                break;
            case READY:
                // If it hits the switch again during normal operation, re-zero it for precision
                if (m_limitReached) {
                    m_leaderMotor.stopMotor(); // we are at the bottom, stop for safety
                    m_state = states.INIT;
                    // initializeMotorConfig is a no-op since we do not use the brake to keep the elevator in place
                    // initializeMotorConfig(false);
                }
        }
    }

    /**
     * Commands the elevator to a specific height using onboard PID.
     * 
     * @param position Target position in converted units (e.g., inches). 
     *                 Clamped between MAX and DOWN constants.
     */
    public void setPosition(double position){
        // set desired position
        // measured in rotations of motor * a constant
        if (m_state == states.READY || m_state == states.READYLOW){
            m_desiredPosition = MathUtil.clamp(position, ElevatorConstants.kMaxElevatorPosition, ElevatorConstants.kDownElevatorPosition);
            m_PIDController.setReference(m_desiredPosition, ControlType.kPosition);
        }
    }

    // applies brake or coast mode to motors based on isBrakeMode parameter
    private void initializeMotorConfig(boolean isBrakeMode) {
        m_leaderConfig.idleMode(isBrakeMode ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
        m_followConfig.idleMode(isBrakeMode ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);

        // Re-apply configurations to motors
        m_leaderMotor.configure(m_leaderConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);
        // reapply to follower
        m_followMotor.configure(m_followConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);
    }
    // resets the relative encoder to zero when the limit switch is triggered
    private void resetEncoder(){
        m_encoder.setPosition(0);
        m_encoderPosition = 0;
    }

    /** Resets the relative encoder to zero; used when the limit switch is triggered. */
    public double getPosition(){
        if (m_state == states.READY || m_state == states.READYLOW){
            return m_encoderPosition;
        } else {
            return -1;
        }
    }

    /** @return The last commanded setpoint. */
    public double getDesiredPosition() {
        return m_desiredPosition;
    }

    /** Immediately cuts power to the elevator motors. */
    public void stopSystem(){
        m_leaderMotor.stopMotor();
    }
}
