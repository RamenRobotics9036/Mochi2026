package robotutils.pub.interfaces.dashboard;

public class DashboardNames {

    //* Constructor cant be called. */
    private DashboardNames() {
        throw new UnsupportedOperationException(
            "This is a constants class and cannot be instantiated");
    }

    // Ground truth simulation provider and its Field2d-capable item names
    public static final String kGroundTruthProviderName = "GroundTruthSimProvider";
    public static final String kGroundTruthPoseItemName = "GroundTruthPose";

    // Per-robot config provider
    public static final String kPerRobotConfigProviderName = "PerRobotConfigProvider";

}
