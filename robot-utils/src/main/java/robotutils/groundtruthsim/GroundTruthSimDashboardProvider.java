package robotutils.groundtruthsim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;
import robotutils.dashboard.Field2dObjectRenderer;
import robotutils.pub.interfaces.dashboard.DashboardProviderInterface;
import robotutils.pub.interfaces.dashboard.DoublePublisherWrapper;
import robotutils.pub.interfaces.dashboard.Pose2dPublisherWrapper;


/** Dashboard provider for ground truth simulation pose. */
public class GroundTruthSimDashboardProvider
    implements DashboardProviderInterface<GroundTruthSimDashboardSettings> {

    public static final String kGroundTruthPoseItemName = "groundTruthPose";
    private boolean m_isInitialized = false;
    private boolean m_isUpdated = false;
    private GroundTruthSimDashboardSettings m_latestSettings = null;
    private Pose2dPublisherWrapper m_groundTruthPosePublisher;
    private DoublePublisherWrapper m_estimateToGroundTruthPublisher;
    private final List<Field2dObjectRenderer> m_field2dRenderers = new ArrayList<>();

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

        m_estimateToGroundTruthPublisher = new DoublePublisherWrapper(
            tableRoot.getDoubleTopic("EstimateToGroundTruth").publish());

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
        m_estimateToGroundTruthPublisher.set(m_latestSettings.poseEstimateToGroundTruthDistance());

        for (Field2dObjectRenderer renderer : m_field2dRenderers) {
            renderer.renderPose(m_latestSettings.groundTruthPose());
        }
    }

    @Override
    public void addCustomRenderer(Field2dObjectRenderer renderer, String providerItemName) {
        if (kGroundTruthPoseItemName.equals(providerItemName)) {
            m_field2dRenderers.add(renderer);
            return;
        }

        throw new IllegalArgumentException(
            "GroundTruthSimDashboardProvider does not support itemName: " + providerItemName);
    }

    @Override
    public void setLatestSettings(GroundTruthSimDashboardSettings settings) {
        m_latestSettings = settings;
        m_isUpdated = true;
    }
}
