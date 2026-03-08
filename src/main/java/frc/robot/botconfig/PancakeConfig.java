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
     * FORCE DISABLE SUBSYSTEMS (e.g. Pancake is missing several)
     *
     ************************************************************************************/

    /** Force disable shooter. */
    @Override
    public boolean shouldForceDisableShooter() {
        return true;
    }

    /** Force disable indexer. */
    @Override
    public boolean shouldForceDisableIndexer() {
        return true;
    }

    /** Force disable spinny. */
    @Override
    public boolean shouldForceDisableSpinny() {
        return true;
    }

    /** Force disable climber. */
    @Override
    public boolean shouldForceDisableClimber() {
        return true;
    }

    /** Force disable intake. */
    @Override
    public boolean shouldForceDisableIntake() {
        return true;
    }

    /** Force disable intake arm. */
    @Override
    public boolean shouldForceDisableIntakeArm() {
        return true;
    }


    /*************************************************************************************
     *
     * VISION
     *
     ************************************************************************************/

    private final List<CameraInfo> m_cameras = List.of(
        new CameraInfo("limelight-fixedii", new Transform3d(
            new Translation3d(
                Units.inchesToMeters(12.75), // + is forward
                Units.inchesToMeters(-0.25),        // + is left
                Units.inchesToMeters(9.75)   // x" deck + 4" mount
            ),
            new Rotation3d(0, Math.toRadians(0), 0)  // x degrees up
        )),
        new CameraInfo("limelight", new Transform3d(
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
