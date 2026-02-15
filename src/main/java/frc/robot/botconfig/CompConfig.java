package frc.robot.botconfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.GeneratedCompConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Adapter that bridges BotConfigInterface to the current constants.
 * For now this delegates directly to GeneratedCompConstants; when a second robot
 * is added, this class can be swapped or extended to select between configs.
 */
public class CompConfig implements BotConfigInterface {

    /*************************************************************************************
     *
     * SWERVEDRIVE GENERATED CONSTANTS
     *
     ************************************************************************************/

    @Override
    public CANBus getCANBus() {
        return GeneratedCompConstants.kCANBus;
    }

    @Override
    public LinearVelocity getSpeedAt12Volts() {
        return GeneratedCompConstants.kSpeedAt12Volts;
    }

    @Override
    public LinearVelocity getSpeedInTeleop() {
        return GeneratedCompConstants.kSpeedInTeleop;
    }

    @Override
    public double getAlignmentErrorMargin() {
        return GeneratedCompConstants.kAlignmentErrorMargin;
    }

    @Override
    public double getSwerveMoveInterruptionSensitivity() {
        return GeneratedCompConstants.kSwerveMoveInterruptionSensitivity;
    }

    @Override
    public double getSwerveTurnInterruptionSensitivity() {
        return GeneratedCompConstants.kSwerveTurnInterruptionSensitivity;
    }

    @Override
    public SwerveDrivetrainConstants getDrivetrainConstants() {
        return GeneratedCompConstants.DrivetrainConstants;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontLeft() {
        return GeneratedCompConstants.FrontLeft;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontRight() {
        return GeneratedCompConstants.FrontRight;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackLeft() {
        return GeneratedCompConstants.BackLeft;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackRight() {
        return GeneratedCompConstants.BackRight;
    }

    @Override
    public CommandSwerveDrivetrain createDrivetrain() {
        return GeneratedCompConstants.createDrivetrain();
    }


    /*************************************************************************************
     *
     * INTAKE CONSTANTS
     *
     ************************************************************************************/

    @Override
    public int getIntakeMotorId() {
        return 30;
    }

    @Override
    public int getIntakeStallLimit() {
        return 40;
    }


    /*************************************************************************************
     *
     * SHOOTERCONSTANTS
     *
     ************************************************************************************/

    @Override
    public int getShooterLeftMotorId() {
        return 40;
    }

    @Override
    public int getShooterRightMotorId() {
        return 41;
    }

    @Override
    public int getShooterStatorCurrentLimit() {
        return 40;
    }

    @Override
    public int getShooterSupplyCurrentLimit() {
        return 50;
    }

    @Override
    public int getShooterHoodPwmChannel() {
        return 0;
    }


    /*************************************************************************************
     *
     * VISION
     *
     ************************************************************************************/

    @Override
    public String getVisionLimelightNameReal() {
        return "limelight-fixed";
    }
}
