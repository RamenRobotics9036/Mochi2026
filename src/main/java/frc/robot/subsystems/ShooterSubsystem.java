package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.sim.RollerSim.TwoMotorRollerIoInterface;
import frc.robot.subsystems.shooter.HoodActuator;

public class ShooterSubsystem extends SubsystemBase {
    private final BotConfigInterface m_configInterface;
    private final TwoMotorRollerIoInterface m_shooterIO;
    private final TwoMotorRollerIoInterface.DeviceOutputs m_shooterOutputs =
        new TwoMotorRollerIoInterface.DeviceOutputs();

    private HoodActuator m_hood;

    /** Creates a new ShooterSubsystem using the ShooterConstants from Constants.java. */
    public ShooterSubsystem(BotConfigInterface configInterface, TwoMotorRollerIoInterface shooterIO, HoodActuator hood) {
        m_configInterface = configInterface;
        m_shooterIO = shooterIO;
        m_hood = hood;
    }

    public void setSpeed(double speed){

        m_shooterIO.setSpeed(speed);
    }

    public void stop(){
        m_shooterIO.stop();

    }

    /**
     * Uses the linear actuator to adjust the hood to the target position.
     * 
     * @param hoodPosition The position of the hood, scaled from 0 to 1 (least to most extended)
     */
    public void setHood(double hoodPosition) {
        m_hood.setPosition(hoodPosition);
    }

    /**
     * Uses the linear actuator to adjust the hood by a target amount.
     * 
     * @param relativePosition The amount to adjust the hood, scaled from 0 to 1
     */
    public void adjustHood(double relativePosition) {
        double currentPosition = m_hood.get();
        double targetPosition = MathUtil.clamp(currentPosition + relativePosition, 0.0, 1.0);

        m_hood.set(targetPosition);
        System.out.println("Current position: "+Double.toString(currentPosition)+", setting to "+Double.toString(targetPosition));
    }

    /**
     * Regularly publishes telemetry to the SmartDashboard for driver and pit feedback.
     */
    @Override
    public void periodic() {
        m_shooterIO.updateOutputs(m_shooterOutputs);
        m_hood.updateCurPos();

        SmartDashboard.putNumber("Shooter/VelocityRPM", m_shooterOutputs.velocityRPM);
    }
}
