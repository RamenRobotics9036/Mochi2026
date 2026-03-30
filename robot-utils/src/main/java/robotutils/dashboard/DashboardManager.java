package robotutils.dashboard;

import java.util.LinkedHashMap;
import java.util.Map;
import robotutils.pub.interfaces.dashboard.DashboardManagerInterface;
import robotutils.pub.interfaces.dashboard.DashboardProviderInterface;

/** Holds all the Dashboard Providers, and calls update() on them every periodic. */
public class DashboardManager implements DashboardManagerInterface {
    Map<String, DashboardProviderInterface<?>> m_providers = new LinkedHashMap<>();

    /** Constructor. */
    public DashboardManager() {
    }

    @Override
    public void update() {
        for (DashboardProviderInterface<?> provider : m_providers.values()) {
            provider.update();
        }
    }

    @Override
    public void registerProvider(String providerName, DashboardProviderInterface<?> provider) {
        if (m_providers.containsKey(providerName)) {
            throw new IllegalArgumentException("Provider is already registered: " + providerName);
        }
        m_providers.put(providerName, provider);
    }
}
