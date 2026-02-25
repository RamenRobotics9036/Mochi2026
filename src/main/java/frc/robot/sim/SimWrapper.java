package frc.robot.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.sim.visionproducers.VisionSimFactory;
import frc.robot.sim.visionproducers.VisionSimInterface;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.auto.AutoLogic;
import java.util.function.Consumer;


/**
 * Unified simulation wrapper combining ground truth physics and vision simulation.
 *
 * <p>This class encapsulates all simulation-specific logic, keeping Robot and
 * RobotContainer free from simulation implementation details.
 *
 * <p>Note that we encapsulated this into a composeable class rather than having
 * the sim class subclass RobotContainer, since its cleaner this way to know
 * that the simulation code is only running under simulation conditions.
 */
public class SimWrapper {
    private final BotConfigInterface m_configInterface;

    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_drivetrain;
    private final GroundTruthSimInterface m_groundTruthSim;
    private final VisionSimInterface m_visionSim;

    /**
     * Creates a new SimWrapper.
     *
     * @param drivetrain The swerve drivetrain
     * @param poseResetConsumer Consumer called when ground truth resets the pose
     *     (typically drivetrain::resetPose)
     */
    public SimWrapper(
            BotConfigInterface configInterface,
            SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain,
            Consumer<Pose2d> poseResetConsumer) {

        if (!Robot.isSimulation()) {
            throw new IllegalStateException("SimWrapper should only be instantiated in simulation");
        }
        if (drivetrain == null) {
            throw new IllegalArgumentException("SwerveDrivetrain cannot be null");
        }
        if (poseResetConsumer == null) {
            throw new IllegalArgumentException("Pose reset consumer cannot be null");
        }

        m_configInterface = configInterface;

        m_drivetrain = drivetrain;

        // Create ground truth simulation
        m_groundTruthSim = GroundTruthSimFactory.create(drivetrain, poseResetConsumer);

        // Create vision simulation
        m_visionSim = VisionSimFactory.create(m_configInterface);
        if (m_visionSim == null) {
            throw new IllegalStateException("VisionSimInterface creation failed");
        }
    }

    /**
     * Must be called by Robot::robotPeriodic.
     * Updates vision simulation (processes camera results and updates pose estimator).
     */
    public void robotPeriodic() {
        m_visionSim.periodic();
    }

    /**
     * Must be called by Robot::simulationPeriodic.
     * Updates physics simulation and vision based on ground truth pose.
     */
    public void simulationPeriodic() {
        var driveState = m_drivetrain.getState();

        // Update ground truth physics simulation
        m_groundTruthSim.simulationPeriodic();

        // Update vision simulation with ground truth pose (not odometry)
        // This ensures cameras see AprilTags based on actual robot position
        Pose2d groundTruthPose = m_groundTruthSim.getGroundTruthPose();
        m_visionSim.simulationPeriodic(groundTruthPose);
    }

    /**
     * Resets the simulated robot to a new pose.
     * Updates both ground truth physics and vision simulation.
     *
     * @param pose The new pose
     */
    public void resetSimPose(Pose2d pose) {
        m_groundTruthSim.resetGroundTruthPoseForSim(pose);
        m_visionSim.resetSimPose(pose);
    }

    /**
     * Proxy call to ground truth sim to inject odometry drift.
     */
    public void injectDrift(double translationOffsetMeters, double rotationOffsetDegrees) {
        m_groundTruthSim.injectDrift(translationOffsetMeters, rotationOffsetDegrees);
    }

    /**
     * Proxy call to ground truth sim to cycle reset position.
     *
     * @param blueAlliancePose The pose to reset to if on blue alliance
     */
    public void cycleResetPosition(Pose2d blueAlliancePose) {
        m_groundTruthSim.cycleResetPosition(blueAlliancePose);
    }

    /**
     * Binds simulation-only POV button actions to the drivetrain.
     *
     * @param driftTrigger    Trigger that injects odometry drift (e.g. povRight)
     * @param resetTrigger    Trigger that cycles the robot reset position (e.g. povLeft)
     * @param drivetrain      The swerve drivetrain
     */
    public void configureSimBindings(
            Trigger driftTrigger,
            Trigger resetTrigger,
            CommandSwerveDrivetrain drivetrain) {

        driftTrigger.onTrue(drivetrain.runOnce(() -> injectDrift(0.5, 15.0)));
        resetTrigger.onTrue(drivetrain.runOnce(() ->
            cycleResetPosition(AutoLogic.getSelectedAutoStartingPose())));
    }

    /**
     * Configures simulation bindings if the provided SimWrapper is non-null.
     * Safe to call unconditionally — no-op when {@code simWrapper} is null.
     *
     * @param simWrapper      The SimWrapper instance, or null when not in simulation
     * @param driftTrigger    Trigger that injects odometry drift (e.g. povRight)
     * @param resetTrigger    Trigger that cycles the robot reset position (e.g. povLeft)
     * @param drivetrain      The swerve drivetrain
     */
    public static void configureSimBindings(
            SimWrapper simWrapper,
            Trigger driftTrigger,
            Trigger resetTrigger,
            CommandSwerveDrivetrain drivetrain) {

        if (simWrapper == null) {
            // Sanity check that simWrapper is only null if !Robot.isSimulation()
            if (Robot.isSimulation()) {
                throw new IllegalStateException("SimWrapper is null in simulation mode");
            }

            return;
        }
        simWrapper.configureSimBindings(driftTrigger, resetTrigger, drivetrain);
    }

    /**
     * Creates a SimWrapper if in simulation, or returns null for real robot.
     *
     * @param configInterface The bot configuration
     * @param drivetrain The swerve drivetrain
     * @param poseResetConsumer Consumer called when ground truth resets the pose
     * @return A new SimWrapper, or null if not in simulation
     */
    public static SimWrapper create(
            BotConfigInterface configInterface,
            SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain,
            Consumer<Pose2d> poseResetConsumer) {

        if (!Robot.isSimulation()) {
            return null;
        }
        return new SimWrapper(configInterface, drivetrain, poseResetConsumer);
    }

    /**  Gets the ground truth pose from the simulation. */
    public Pose2d getGroundTruthPose() {
        return m_groundTruthSim.getGroundTruthPose();
    }

    /**
     * Get the simulation debug Field2d for visualization.
     *
     * @return The VisionSystemSim's debug field, or null if not in simulation
     */
    public Field2d getSimDebugField() {
        // This should ONLY be called when we're in simulation, since PhotonVision sim field is
        // only created in simulation.
        if (!Robot.isSimulation()) {
            throw new IllegalStateException("getSimDebugField should only be in simulation mode");
        }

        return m_visionSim.getSimDebugField();
    }
}
