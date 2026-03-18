package frc.robot.sim;

/**
 * Manages simulated hardware faults for autonomous testing.
 * Use this to enable/disable specific modeled imperfections (e.g. drivetrain pull)
 * so that vision correction and path following can be validated.
 */
public class FaultyAutoSim {

    private final GroundTruthSimInterface m_groundTruthSim;

    /**
     * Constructs a FaultyAutoSim.
     *
     * @param groundTruthSim The ground truth sim to apply faults to
     */
    public FaultyAutoSim(GroundTruthSimInterface groundTruthSim) {
        m_groundTruthSim = groundTruthSim;
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
     * Resets all simulated auto faults to their default (disabled) state.
     */
    public void resetAllAutoSimFaults() {
        enablePullRight(false);
    }
}
