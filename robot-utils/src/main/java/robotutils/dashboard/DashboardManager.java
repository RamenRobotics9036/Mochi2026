package robotutils.dashboard;

import java.util.ArrayList;
import java.util.List;
import robotutils.pub.interfaces.dashboard.DashboardManagerInterface;
import robotutils.pub.interfaces.dashboard.DashboardProviderInterface;

/** Holds all the Dashboard Providers, and calls update() on them every periodic. */
public class DashboardManager implements DashboardManagerInterface {
    List<DashboardProviderInterface<?>> m_providers = new ArrayList<>();

    /** Constructor. */
    public DashboardManager() {
    }

    @Override
    public void update() {
        for (DashboardProviderInterface<?> provider : m_providers) {
            provider.update();
        }
    }

    @Override
    public void registerProvider(DashboardProviderInterface<?> provider) {
        m_providers.add(provider);
    }
}
