package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpinnyWheelsConstants;

public class SpinnyWheels extends SubsystemBase {
    public SpinnyWheels(frc.robot.sim.RollerSim.RollerIoInterface io) {
    }

    /**
     * Spins the wheels at the constant speed defined in Constants.
     */
    public void spin() {
    }

    public void stop() {
    }

    @Override
    public void periodic() {
        // Monitors to ensure the motor is actually spinning
        SmartDashboard.putNumber("Spinny/Current RPM", m_encoder.getVelocity());
    }
}