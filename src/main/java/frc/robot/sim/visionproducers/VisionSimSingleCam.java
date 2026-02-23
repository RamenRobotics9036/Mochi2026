package frc.robot.sim.visionproducers;

import static frc.robot.sim.visionproducers.VisionSimConstants.Vision.*;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;
import frc.robot.sim.visionproducers.VisionSimConstants.Vision;


/** Handles a single PhotonVision camera. */
public class VisionSimSingleCam {
    private final String m_camName;
    private final Transform3d m_robotToCam;

    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_photonEstimator;

    // Limelight NetworkTables publisher
    private final LimelightTablePublisher m_limelightPublisher;

    /** Constructor. */
    public VisionSimSingleCam(String camName, Transform3d robotToCam) {
        m_camName = camName;
        m_robotToCam = robotToCam;

        m_camera = new PhotonCamera(m_camName);

        m_photonEstimator = new PhotonPoseEstimator(
            kTagLayout,
            m_robotToCam);

        m_limelightPublisher = new LimelightTablePublisher(
            m_camName);
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
}
