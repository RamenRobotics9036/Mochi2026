package robotutils.pub.interfaces.dashboard;

import edu.wpi.first.networktables.StringPublisher;
import java.util.Objects;

/** Wraps a StringPublisher and only publishes when the value changes. */
public class StringPublisherWrapper {
    private final StringPublisher m_stringPublisher;
    private String m_lastValue = null;
    private boolean m_lastValueSet = false;

    /** Constructor. */
    public StringPublisherWrapper(StringPublisher stringPublisher) {
        m_stringPublisher = stringPublisher;
    }

    /** Updates the published value if it changed from the last value. */
    public void set(String newValue) {
        if (m_lastValueSet && Objects.equals(m_lastValue, newValue)) {
            return;
        }

        m_lastValue = newValue;
        m_lastValueSet = true;
        m_stringPublisher.set(newValue);
    }
}
