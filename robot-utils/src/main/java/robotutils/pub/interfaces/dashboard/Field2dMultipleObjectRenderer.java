package robotutils.pub.interfaces.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.Arrays;

/** Renders a fixed-size array of Pose2d objects onto a named object within a Field2d. */
public class Field2dMultipleObjectRenderer {
    private final Field2d m_field;
    private final String m_fieldObjectName;
    private final int m_expectedCount;

    // Pre-allocated to avoid runtime allocations. Pose2d is immutable, so storing
    // references here is safe (no deep clone needed).
    private final Pose2d[] m_lastMultipleValue;
    private boolean m_lastValueSet = false;
    private boolean m_lastWasNull = false;

    /**
     * Constructor.
     *
     * @param field the Field2d to render onto
     * @param fieldObjectName the named object within the field
     * @param expectedCount the exact number of poses expected on every call (e.g. 4 for swerve
     *     wheels); the array is pre-allocated here to avoid runtime allocations
     */
    public Field2dMultipleObjectRenderer(Field2d field, String fieldObjectName, int expectedCount) {
        if (field == null) {
            throw new IllegalArgumentException("field cannot be null");
        }
        if (fieldObjectName == null || fieldObjectName.isBlank()) {
            throw new IllegalArgumentException("fieldObjectName cannot be blank");
        }
        if (expectedCount <= 0) {
            throw new IllegalArgumentException("expectedCount must be positive");
        }

        m_field = field;
        m_fieldObjectName = fieldObjectName;
        m_expectedCount = expectedCount;
        m_lastMultipleValue = new Pose2d[expectedCount];
    }

    /**
     * Draws the poses on the configured field object, or clears it when poses is null.
     *
     * @param poses the poses to render; must have exactly {@code expectedCount} elements, or be
     *     null to clear
     * @throws IllegalArgumentException if poses.length != expectedCount
     */
    public void renderMultiplePoses(Pose2d[] poses) {
        // Poses == null means that we just want to REMOVE the field of these poses
        if (poses == null) {
            if (!m_lastWasNull) {
                m_lastValueSet = false;
                m_lastWasNull = true;
                m_field.getObject(m_fieldObjectName).setPoses();
            }
            return;
        }
        m_lastWasNull = false;

        if (poses.length != m_expectedCount) {
            throw new IllegalArgumentException(
                    "poses.length must be "
                            + m_expectedCount
                            + " but got "
                            + poses.length);
        }

        // Skip update if nothing changed
        if (m_lastValueSet && Arrays.equals(m_lastMultipleValue, poses)) {
            return;
        }

        // Copy references into pre-allocated array (Pose2d is immutable, refs are safe)
        System.arraycopy(poses, 0, m_lastMultipleValue, 0, m_expectedCount);
        m_lastValueSet = true;

        m_field.getObject(m_fieldObjectName).setPoses(m_lastMultipleValue);
    }
}
