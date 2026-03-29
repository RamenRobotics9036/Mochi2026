package robotutils.perrobotconfig;

import robotutils.pub.interfaces.DashboardProviderInterface;

/** Dashboard provider for per-robot configuration values. */
public class PerRobotConfigDashboardProvider implements DashboardProviderInterface {

    /** Constructor. */
    public PerRobotConfigDashboardProvider() {
    }

    @Override
    public void init() {
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }

    @Override
    public void update() {
        throw new UnsupportedOperationException("Unimplemented method 'update'");
    }

    /** Caller should use this when new settings are available. */
    public void setLatestSettings(PerRobotConfigDashboardSettings settings) {
    }
}
