package frc.robot.botconfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Robot;
import frc.robot.botconfig.BotConfigInterface.CameraInfo;
import frc.robot.generated.GeneratedCompConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.List;

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
    public String getConfigName() {
        return "Competition Bot Config";
    }

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
     * VISION
     *
     ************************************************************************************/

    private final List<CameraInfo> m_cameras = List.of(
        new CameraInfo("limelight", new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-0.5),  // 1/2 inch back
                Units.inchesToMeters(4.0),   // 4 inches left of center
                Units.inchesToMeters(16.5)   // 12.5" deck + 4" mount
            ),
            new Rotation3d(0, Math.toRadians(-23), 0)  // 23 degrees up
        )),
        new CameraInfo("limelight-back", new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11.75),         // + is forward
                Units.inchesToMeters(1.75),   // + is left
                Units.inchesToMeters(8.75)    // x" deck + 4" mount
            ),
            new Rotation3d(0, Math.toRadians(-30), Math.toRadians(180))  // x degrees up, backwards
        ))
    );

    @Override
    public List<CameraInfo> getCameras() {
        return m_cameras;
    }
}
