package frc.robot.botconfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.GeneratedPancakeConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.List;

/**
 * Adapter that bridges BotConfigInterface to the current constants.
 * For now this delegates directly to GeneratedPancakeConstants; when a second robot
 * is added, this class can be swapped or extended to select between configs.
 */
public class PancakeConfig implements BotConfigInterface {

    /*************************************************************************************
     *
     * SWERVEDRIVE GENERATED CONSTANTS
     *
     ************************************************************************************/

    @Override
    public String getConfigName() {
        return "Pancake Bot Config";
    }

    @Override
    public CANBus getCANBus() {
        return GeneratedPancakeConstants.kCANBus;
    }

    @Override
    public LinearVelocity getSpeedAt12Volts() {
        return GeneratedPancakeConstants.kSpeedAt12Volts;
    }

    @Override
    public LinearVelocity getSpeedInTeleop() {
        return GeneratedPancakeConstants.kSpeedInTeleop;
    }

    @Override
    public double getAlignmentErrorMargin() {
        return GeneratedPancakeConstants.kAlignmentErrorMargin;
    }

    @Override
    public double getSwerveMoveInterruptionSensitivity() {
        return GeneratedPancakeConstants.kSwerveMoveInterruptionSensitivity;
    }

    @Override
    public double getSwerveTurnInterruptionSensitivity() {
        return GeneratedPancakeConstants.kSwerveTurnInterruptionSensitivity;
    }

    @Override
    public SwerveDrivetrainConstants getDrivetrainConstants() {
        return GeneratedPancakeConstants.DrivetrainConstants;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontLeft() {
        return GeneratedPancakeConstants.FrontLeft;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontRight() {
        return GeneratedPancakeConstants.FrontRight;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackLeft() {
        return GeneratedPancakeConstants.BackLeft;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackRight() {
        return GeneratedPancakeConstants.BackRight;
    }

    @Override
    public CommandSwerveDrivetrain createDrivetrain() {
        return GeneratedPancakeConstants.createDrivetrain();
    }


    /*************************************************************************************
     *
     * VISION
     *
     ************************************************************************************/

    private final List<CameraInfo> m_cameras = List.of(
        new CameraInfo("limelight-fixed", new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-0.5),  // 1/2 inch back
                Units.inchesToMeters(4.0),   // 4 inches left of center
                Units.inchesToMeters(16.5)   // 12.5" deck + 4" mount
            ),
            new Rotation3d(0, Math.toRadians(-23), 0)  // 23 degrees up
        )),
        new CameraInfo("liimelight-fixed2", new Transform3d(
            new Translation3d(-0.5, 0.0, 0.5),
            new Rotation3d(0, 0, Math.PI)
        ))
    );

    @Override
    public List<CameraInfo> getCameras() {
        return m_cameras;
    }

    @Override
    public String getCameraName(int cameraNum) {
        if (cameraNum < 0 || cameraNum >= m_cameras.size()) {
            throw new IllegalArgumentException("Invalid camera number: " + cameraNum);
        }
        return m_cameras.get(cameraNum).cameraName;
    }

    @Override
    public Transform3d getRobotToCam(int cameraNum) {
        if (cameraNum < 0 || cameraNum >= m_cameras.size()) {
            throw new IllegalArgumentException("Invalid camera number: " + cameraNum);
        }
        return m_cameras.get(cameraNum).robotToCam;
    }
}
