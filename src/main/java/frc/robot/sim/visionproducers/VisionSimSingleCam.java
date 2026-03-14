package frc.robot.sim.visionproducers;

import static frc.robot.sim.visionproducers.VisionSimConstants.Vision.*;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;


/** Handles a single PhotonVision camera. */
public class VisionSimSingleCam {
    private final String m_photonCamName;
    private final String m_limelightCamName;
    private final Transform3d m_robotToCam;

    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_photonEstimator;

    // Limelight NetworkTables publisher
    private final LimelightTablePublisher m_limelightPublisher;

    /** Constructor. */
    public VisionSimSingleCam(
        String photonCamName,
        String limelightCamName,
        Transform3d robotToCam) {

        m_photonCamName = photonCamName;
        m_limelightCamName = limelightCamName;
        m_robotToCam = robotToCam;

        m_camera = new PhotonCamera(m_photonCamName);

        m_photonEstimator = new PhotonPoseEstimator(
            kTagLayout,
            m_robotToCam);

        m_limelightPublisher = new LimelightTablePublisher(
            m_limelightCamName);
    }

    /** Create a simulated camera and return it. */
    public void addToVisionSystem(VisionSystemSim visionSystemSim, SimCameraProperties cameraProp) {
        PhotonCameraSim cameraSim;

        if (!Robot.isSimulation()) {
            throw new IllegalStateException(
                "Camera simulation should only be created in simulation");
        }

        // Create a PhotonCameraSim which will update the linked PhotonCamera's values
        // with visible targets.
        cameraSim = new PhotonCameraSim(m_camera, cameraProp);
        // Set realistic detection range limits
        cameraSim.setMinTargetAreaPixels(kMinTargetAreaPixels);
        cameraSim.setMaxSightRange(kMaxSightRangeMeters);

        visionSystemSim.addCamera(cameraSim, m_robotToCam);

        // $TODO - Double check that both wireframes should be drawn
        cameraSim.enableDrawWireframe(true);
    }

    /**
     * Gets the latest PhotonVision pose, and publishes it to Network Tables
     * for limelighthelper to read.
     */
    public void processCamera() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : m_camera.getAllUnreadResults()) {
            visionEst = m_photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = m_photonEstimator.estimateLowestAmbiguityPose(result);
            }

            // Publish to Limelight NetworkTables for MultiCamOdometry to consume
            LimelightData data = PhotonToLimelightConverter.convertPipelineResult(
                result,
                m_robotToCam);
            double totalLatencyMs = data.pipelineLatencyMs + data.captureLatencyMs;
            PhotonToLimelightConverter.convertBotpose(
                visionEst.map(est -> est.estimatedPose).orElse(null),
                result.getTargets(),
                m_robotToCam,
                totalLatencyMs,
                data);
            m_limelightPublisher.publish(data);
        }
    }
}
