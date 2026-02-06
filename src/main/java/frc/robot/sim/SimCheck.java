package frc.robot.sim;

import frc.robot.Robot;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.SimConstants.SimMode;

/** Specialized checks so we can run in Simulation with varying degrees
 * of simulated features.
 */
public class SimCheck {

    public static SimMode getSimMode() {
        return SimConstants.currentMode;
    }

    /** Includes JUST THE BASICS in simulation.  This includes:
     * - Limelight tables have simulated vision data
     * - Ground truth pose simulation is on, since cameras are placed on that
     * - Dashboards show: Robot pose on field, ground truth pose on field
     * (getSimDebugField() should not be called, since we shouldnt add extra
     *  visualizations to it in this mode)
     */
    public static boolean isSimulation() {
        return Robot.isSimulation();
    }

    public static boolean isSimulationDebug() {
        if (!Robot.isSimulation()) {
            return false;
        }

        return SimConstants.currentMode == SimMode.SIM_DEBUG;
    }
}
