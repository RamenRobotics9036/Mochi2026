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

@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class ElevatorSystem extends SubsystemBase{
    //Motors are on opposite sides of a shaft
    private final SparkMax m_leaderMotor = new SparkMax(ElevatorConstants.kLeaderMotorID, MotorType.kBrushless);
    private final SparkMax m_followMotor = new SparkMax(ElevatorConstants.kFollowMotorID, MotorType.kBrushless);
    private SparkMaxConfig m_leaderConfig = new SparkMaxConfig();
    private SparkMaxConfig m_followConfig = new SparkMaxConfig();
    private RelativeEncoder m_encoder = m_leaderMotor.getEncoder();
    private SparkClosedLoopController m_PIDController = m_leaderMotor.getClosedLoopController();
    //Cached values
    private double m_desiredPosition;
    private double m_encoderPosition;
    private boolean m_limitReached = false;

    private DigitalInput m_limitSwitch= new DigitalInput(ElevatorConstants.kDIOIndex);
    /* Sensor will reset a relative encoder when the elevator lowers fully
     * Encoder will be used to prevent arm from going too high*/

    enum states {
        INIT,
        READYLOW,
        READY,
    }
    private states m_state = states.INIT;
    
    public ElevatorSystem() {
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig
            .p(4)
            .i(0)
            .d(0);
        closedLoopConfig.positionWrappingEnabled(false);
        closedLoopConfig.outputRange(-ElevatorConstants.elevatorMaxSpeed, ElevatorConstants.elevatorMaxSpeed);
        //closedLoopConfig.minOutput();
        //closedLoopConfig.maxOutput();
        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(ElevatorConstants.kRotationToElevatorRatio);
        // encoderConfig.velocityConversionFactor(ElevatorConstants.kRotationToElevatorRatio / 60);

        m_leaderConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_leaderConfig.smartCurrentLimit(IntakeConstants.kStallLimit);
        m_leaderConfig.inverted(true);
        m_leaderConfig.apply(closedLoopConfig);
        m_leaderConfig.apply(encoderConfig);
        m_leaderMotor.configure(m_leaderConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);

        m_followConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_followConfig.smartCurrentLimit(IntakeConstants.kStallLimit);
        m_followConfig.inverted(false);
        m_followConfig.follow(ElevatorConstants.kLeaderMotorID);
        m_followMotor.configure(m_followConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);

        m_encoderPosition = m_encoder.getPosition();

        initShuffleboard();
    }

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

    @Override
    public void periodic(){
        // update cached values
        m_limitReached = m_limitSwitch.get();
        m_encoderPosition = m_encoder.getPosition();

        switch (m_state) {
        
            case INIT:
                if (m_limitReached) {
                    m_state = states.READYLOW;
                    resetEncoder();
                    // initializeMotorConfig is a no-op since we do not use the brake to keep the elevator in place
                    // initializeMotorConfig(false); // Set to false so we can set the elevator down
                }
                break;
            case READYLOW:
                if (!m_limitReached) {
                    m_state = states.READY;
                }
                break;
            case READY:
                if (m_limitReached) {
                    m_leaderMotor.stopMotor(); // we are at the bottom, stop for safety
                    m_state = states.INIT;
                    // initializeMotorConfig is a no-op since we do not use the brake to keep the elevator in place
                    // initializeMotorConfig(false);
                }
        }
    }

    public void setPosition(double position){
        // set desired position
        // measured in rotations of motor * a constant
        if (m_state == states.READY || m_state == states.READYLOW){
            m_desiredPosition = MathUtil.clamp(position, ElevatorConstants.kMaxElevatorPosition, ElevatorConstants.kDownElevatorPosition);
            m_PIDController.setReference(m_desiredPosition, ControlType.kPosition);
        }
    }

    private void initializeMotorConfig(boolean isBrakeMode) {
        m_leaderConfig.idleMode(isBrakeMode ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
        m_followConfig.idleMode(isBrakeMode ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);

        m_leaderMotor.configure(m_leaderConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);
        m_followMotor.configure(m_followConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);
    }

    private void resetEncoder(){
        m_encoder.setPosition(0);
        m_encoderPosition = 0;
    }

    public double getPosition(){
        if (m_state == states.READY || m_state == states.READYLOW){
            return m_encoderPosition;
        } else {
            return -1;
        }
    }

    public double getDesiredPosition() {
        return m_desiredPosition;
    }

    public void stopSystem(){
        m_leaderMotor.stopMotor();
    }
}
