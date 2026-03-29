package robotutils.pub.interfaces.dashboard;

/**
 * Interface for a component to provide data to dashboard/networktables.
 * Basic model we use here: Each DashboardProvider has methods for a component to
 * update values.  Those values are cached in the DashboardProvider and pushed to
 * NetworkTables on Update().
 */
public interface DashboardProviderInterface<T> {

    /** Root name for NetworkTables entries. */
    static String getNetworkTableRoot() {
        return "RobotUtils";
    }

    /** Dashboard provider should add the entries to NetworkTables in Init(). */
    void init();

    /** Dashboard provider should update NetworkTables with latest cached values. */
    void update();

    /** Update the latest settings that we cache to display on next periodic update. */
    public void setLatestSettings(T settings);
}
