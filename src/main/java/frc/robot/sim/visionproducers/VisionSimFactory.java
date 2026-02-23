package frc.robot.sim.visionproducers;

import frc.robot.Robot;
import frc.robot.botconfig.BotConfigInterface;

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
    public static VisionSimInterface create(BotConfigInterface configInterface) {
        if (Robot.isSimulation()) {
            return new VisionSim(configInterface);
        }
        return null;
    }
}
