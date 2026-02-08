package frc.robot.sim.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.List;
import java.util.Optional;

/**
 * Displays a pose on a Field2d using one of several named objects.
 * All objects are cleared first, then the pose is set on the object at the given index.
 * This allows different field object appearances (e.g. color) based on state.
 */
public class ShowIcon {
    private final List<String> m_objectNames;

    /**
     * Creates a new ShowIcon.
     *
     * @param objectNames The list of field object names (each can have a different appearance)
     */
    public ShowIcon(List<String> objectNames) {
        if (objectNames == null || objectNames.isEmpty()) {
            throw new IllegalArgumentException("objectNames must be non-empty");
        }
        m_objectNames = List.copyOf(objectNames);
    }

    /**
     * Shows or hides an icon on the given field.
     * All objects are cleared first, then the pose is set on the object at showIndex.
     *
     * @param field The field to show the icon on
     * @param pose The pose to display, or empty to hide all objects
     * @param showIndex Index into objectNames selecting which object to display
     */
    public void show(Optional<Field2d> field, Optional<Pose2d> pose, int showIndex) {
        field.ifPresent(f -> {
            // Clear all objects first
            for (String name : m_objectNames) {
                f.getObject(name).setPoses();
            }

            // Show on the selected object
            pose.ifPresent(p -> f.getObject(m_objectNames.get(showIndex)).setPose(p));
        });
    }
}
