package robotutils.pub.interfaces.dashboard;

/** Interface for dashboard manager. */
public interface DashboardManagerInterface {

    /** Called periodically to update all dashboard providers. */
    void update();

    /**
     * After a provider is initialized, it can be registered with the dashboard manager.
     * The dashboard manager will call update() on each registered provider.
     */
    void registerProvider(String providerName, DashboardProviderInterface<?> provider);
}
