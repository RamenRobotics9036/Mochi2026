package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Consumer;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;


/** Wrapper used by RobotContainer class for simulation.  Prefer composition over inheritance,
 * since nobody wants their main subsystems to inherit from simulation classes.
 *
 * WrapperSimRobotContainer owns:
 * - NA
 *
 * It holds references to:
 * - NA
 */
public class WrapperSimRobotContainer {
    public GroundTruthSimInterface m_groundTruthSim;
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_drivetrain;
    private Consumer<Pose2d> m_visionResetter;
    private Consumer<Pose2d> m_poseResetConsumer;

    /** Constructor. */
    public WrapperSimRobotContainer(
        SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain,
        Consumer<Pose2d> poseResetConsumer) {

        if (drivetrain == null) {
            throw new IllegalArgumentException("SwerveDrivetrain cannot be null");
        }
        if (poseResetConsumer == null) {
            throw new IllegalArgumentException("Pose reset consumer cannot be null");
        }

        m_drivetrain = drivetrain;
        m_poseResetConsumer = poseResetConsumer;

        // Ground truth simulation setup
        m_groundTruthSim = GroundTruthSimFactory.create(drivetrain, m_poseResetConsumer);
    }

    /**
     * Sets the vision resetter consumer to be called when the robot pose is reset.
     *
     * @param resetter Consumer that accepts a Pose2d to reset vision position
     */
    public void setVisionResetter(Consumer<Pose2d> resetter) {
        m_visionResetter = resetter;
    }

    public void resetSimRobotPose(Pose2d pose) {
        m_groundTruthSim.resetGroundTruthPoseForSim(pose);
        m_visionResetter.accept(pose);
    }

    public GroundTruthSimInterface getGroundTruthSim() {
        return m_groundTruthSim;
    }
}
