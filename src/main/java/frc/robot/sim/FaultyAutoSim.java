package frc.robot.sim;

import edu.wpi.first.math.geometry.Transform3d;
import robotutils.interfaces.VisionSimInterface;

/**
 * Manages simulated hardware faults for autonomous testing.
 * Use this to enable/disable specific modeled imperfections (e.g. drivetrain pull)
 * so that vision correction and path following can be validated.
 */
public class FaultyAutoSim {

    private final GroundTruthSimInterface m_groundTruthSim;
    private final VisionSimInterface m_visionSim;

    /**
     * Constructs a FaultyAutoSim.
     *
     * @param groundTruthSim The ground truth sim to apply faults to
     * @param visionSim The vision sim to apply camera faults to
     */
    public FaultyAutoSim(GroundTruthSimInterface groundTruthSim, VisionSimInterface visionSim) {
        m_groundTruthSim = groundTruthSim;
        m_visionSim = visionSim;
    }

    /**
     * Enables or disables simulated rightward pull during forward motion.
     *
     * @param enabled true to enable pull-right, false to disable
     */
    public void enablePullRight(boolean enabled) {
        m_groundTruthSim.enablePullRight(enabled);
    }

    /**
     * Enables or disables simulated clockwise rotation drift during turning.
     *
     * @param enabled true to enable clockwise rotation drift, false to disable
     */
    public void enableRotateClockwise(boolean enabled) {
        m_groundTruthSim.enableRotateClockwise(enabled);
    }

    /**
     * Offsets the primary camera's simulated physical position to model miscalibration.
     * The pose estimator is unaffected — only where the sim places the camera changes.
     *
     * @param offset Additional transform to apply on top of the static mounting offset
     */
    public void enableCameraMisplaced(Transform3d offset) {
        m_visionSim.enablePrimaryCameraMisplaced(offset);
    }

    /**
     * Resets all simulated auto faults to their default (disabled) state.
     */
    public void resetAllAutoSimFaults() {
        enablePullRight(false);
        enableRotateClockwise(false);
        enableCameraMisplaced(new Transform3d());
    }
}
