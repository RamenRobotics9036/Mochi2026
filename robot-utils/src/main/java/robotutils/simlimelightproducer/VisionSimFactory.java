package robotutils.simlimelightproducer;

import edu.wpi.first.wpilibj.RobotBase;
import robotutils.interfaces.CameraInfoList;

/**
 * Factory for creating VisionSimInterface instances.
 * Returns null when not in simulation mode.
 */
public class VisionSimFactory {

    /**
     * Creates a VisionSimInterface instance if running in simulation mode.
     *
     * @return A VisionSimInterface instance, or null if not in simulation
     */
    public static VisionSimInterface create(CameraInfoList cameras) {
        if (RobotBase.isSimulation()) {
            return new VisionSim(cameras);
        }
        return null;
    }
}
