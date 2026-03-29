package robotutils.groundtruthsim;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import robotutils.pub.interfaces.dashboard.DashboardProviderInterface;
import robotutils.pub.interfaces.dashboard.Pose2dPublisherWrapper;

/** Dashboard provider for ground truth simulation pose. */
public class GroundTruthSimDashboardProvider
    implements DashboardProviderInterface<GroundTruthSimDashboardSettings> {

    private boolean m_isInitialized = false;
    private boolean m_isUpdated = false;
    private GroundTruthSimDashboardSettings m_latestSettings = null;
    private Pose2dPublisherWrapper m_groundTruthPosePublisher;

    /** Constructor. */
    public GroundTruthSimDashboardProvider() {
    }

    @Override
    public void init() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable tableRoot = inst.getTable(
                DashboardProviderInterface.getNetworkTableRoot());

        m_groundTruthPosePublisher = new Pose2dPublisherWrapper(
            tableRoot.getStructTopic("GroundTruthPose", Pose2d.struct).publish());

        m_isInitialized = true;
    }

    @Override
    public void update() {
        if (!m_isInitialized) {
            throw new IllegalStateException("GroundTruthSimDashboardProvider not initialized");
        }

        // It's OK if they haven't yet updated the values, we just skip
        if (!m_isUpdated) {
            return;
        }

        m_groundTruthPosePublisher.set(m_latestSettings.groundTruthPose());
    }

    @Override
    public void setLatestSettings(GroundTruthSimDashboardSettings settings) {
        m_latestSettings = settings;
        m_isUpdated = true;
    }
}
