package frc.robot.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.generated.TunerConstants;

/** Wrapper used by Robot class for simulation.  Prefer composition over inheritance,
 * since nobody wants their main subsystems to inherit from simulation classes.
 *
 * WrapperSimRobot owns:
 * - VisionSim object that it creates
 *
 * It holds references to:
 * - WrapperSimRobotContainer
 */
public class WrapperSimRobot {
    private final WrapperSimRobotContainer m_wrapperRobotContainer;
    private final VisionSimInterface.EstimateConsumer m_visionPoseConsumer;
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_drivetrain;
    private final VisionSimInterface m_visionSim;

    /** Constructor. */
    public WrapperSimRobot(
        WrapperSimRobotContainer wrapperRobotContainer,
        VisionSimInterface.EstimateConsumer visionPoseConsumer,
        SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain) {

        if (wrapperRobotContainer == null) {
            throw new IllegalArgumentException("WrapperSimRobotContainer cannot be null");
        }
        if (visionPoseConsumer == null) {
            throw new IllegalArgumentException("EstimateConsumer cannot be null");
        }
        if (drivetrain == null) {
            throw new IllegalArgumentException("SwerveDrivetrain cannot be null");
        }

        m_wrapperRobotContainer = wrapperRobotContainer;
        m_visionPoseConsumer = visionPoseConsumer;
        m_drivetrain = drivetrain;

        m_visionSim = createVisionSimObject(
            m_wrapperRobotContainer,
            m_visionPoseConsumer);
    }

    private VisionSimInterface createVisionSimObject(
        WrapperSimRobotContainer wrapperRobotContainer,
        VisionSimInterface.EstimateConsumer visionPoseConsumer) {

        VisionSimInterface visionSim;

        // Vision simulation setup
        visionSim = VisionSimFactory.create();
        if (visionSim == null) {
            throw new IllegalStateException("VisionSimInterface creation failed");
        }
        visionSim.subscribePoseEstimates(visionPoseConsumer);

        // Set the vision resetter so pose resets also reset vision simulation
        wrapperRobotContainer.setVisionResetter(visionSim::resetSimPose);

        return visionSim;
    }

    /** Must be called by Robot::robotPeriodic. */
    public void robotPeriodic() {
        // Update vision simulation (processes camera results and updates pose estimator)
        m_visionSim.periodic();
    }

    /**  Must be called by Robot::simulationPeriodic. */
    public void simulationPeriodic() {
        var driveState = m_drivetrain.getState();
        var robotPoseHoldingCamera = driveState.Pose;

        // Ground truth simulation
        GroundTruthSimInterface groundTruthSim = m_wrapperRobotContainer.getGroundTruthSim();
        groundTruthSim.simulationPeriodic();

        // Update vision simulation with the ground truth robot pose (from physics)
        // This ensures the camera sees AprilTags based on where the robot actually is,
        // not where odometry thinks it is. This allows testing of vision correction.
        robotPoseHoldingCamera = groundTruthSim.getGroundTruthPose();

        // Vision simulation update
        if (m_visionSim != null) {
            m_visionSim.simulationPeriodic(robotPoseHoldingCamera);
        }

        // Debug field visualization
        var debugField = m_visionSim != null ? m_visionSim.getSimDebugField() : null;
        if (debugField != null) {
            // Show the estimated pose (what odometry thinks)
            debugField.getObject("EstimatedRobot").setPose(driveState.Pose);
            debugField.getObject("EstimatedRobotModules").setPoses(getModulePoses(driveState));

            // Show the ground truth pose (where the robot actually is in simulation)
            debugField.getObject("GroundTruthRobot").setPose(
                groundTruthSim.getGroundTruthPose());
        }
    }

  /**
   * Get the Pose2d of each swerve module based on the current robot pose and module states.
   */
  private Pose2d[] getModulePoses(SwerveDrivetrain.SwerveDriveState driveState) {
    // Module locations relative to robot center (from TunerConstants)
    Translation2d[] moduleLocations = {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };

    Pose2d[] modulePoses = new Pose2d[4];
    for (int i = 0; i < 4; i++) {
      modulePoses[i] = driveState.Pose.transformBy(
          new Transform2d(moduleLocations[i], driveState.ModuleStates[i].angle)
      );
    }
    return modulePoses;
  }
}
