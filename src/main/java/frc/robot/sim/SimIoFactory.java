package robotutils;

import frc.robot.Constants;
import robotutils.interfaces.ArmIoInterface;
import robotutils.interfaces.ElevatorIoInterface;
import robotutils.interfaces.RollerIoInterface;
import robotutils.interfaces.TwoMotorRollerIoInterface;

/** Helps create specific IO Sim instances for our specific robot. */
public class SimIoFactory {
    private static final RobotUtilsFactory m_robotUtilsFactory = new RobotUtilsFactory();

    /**
     * Creates the shooter IO: a {@link TwoMotorRollerIoSim} in simulation.
     *
     * @return A new {@link TwoMotorRollerIoSim} configured with shooter sim constants.
     */
    public static TwoMotorRollerIoInterface createShooterIoSim() {
        return m_robotUtilsFactory.createTwoMotorRollerIoSim(
            Constants.SimShooterConstants.kDeviceName,
            Constants.SimShooterConstants.kMoiKgM2,
            Constants.ShooterConstants.kShooterGearRatio);
    }

    /**
     * Creates the indexer IO: a {@link RollerIoSim} in simulation.
     *
     * @return A new {@link RollerIoSim} configured with indexer sim constants.
     */
    public static RollerIoInterface createIndexerIoSim() {
        return m_robotUtilsFactory.createRollerIoSim(
            Constants.SimIndexerConstants.kDeviceName,
            Constants.SimIndexerConstants.kMoiKgM2,
            Constants.IndexerConstants.kIndexerGearRatio);
    }

    /**
     * Creates the spinny wheels IO: a {@link RollerIoSim} in simulation.
     *
     * @return A new {@link RollerIoSim} configured with spinny wheels sim constants.
     */
    public static RollerIoInterface createSpinnyIoSim() {
        return m_robotUtilsFactory.createRollerIoSim(
            Constants.SimSpinnyWheelsConstants.kDeviceName,
            Constants.SimSpinnyWheelsConstants.kMoiKgM2,
            Constants.SpinnyWheelsConstants.kSpinGearRatio);
    }

    /**
     * Creates the climber IO: an {@link ElevatorIoSim} in simulation.
     *
     * @return A new {@link ElevatorIoSim} configured with climber sim constants.
     */
    public static ElevatorIoInterface createClimberIoSim() {
        return m_robotUtilsFactory.createElevatorIoSim(
            Constants.SimClimberConstants.kDeviceName,
            Constants.SimClimberConstants.kGearRatio,
            Constants.SimClimberConstants.kCarriageMassKg,
            Constants.SimClimberConstants.kDrumRadiusMeters,
            Constants.SimClimberConstants.kMinHeightMeters,
            Constants.SimClimberConstants.kMaxHeightMeters);
    }

    /**
     * Creates the intake IO: a {@link RollerIoSim} in simulation.
     *
     * @return A new {@link RollerIoSim} configured with intake sim constants.
     */
    public static RollerIoInterface createIntakeIoSim() {
        return m_robotUtilsFactory.createRollerIoSim(
            Constants.SimIntakeConstants.kDeviceName,
            Constants.SimIntakeConstants.kMoiKgM2,
            Constants.IntakeConstants.kIntakeRollerGearRatio);
    }

    /**
     * Creates the intake arm IO: an {@link ArmIoSim} in simulation.
     *
     * @return A new {@link ArmIoSim} configured with intake arm sim constants.
     */
    public static ArmIoInterface createIntakeArmIoSim(
        double minArmAngleDegrees,
        double maxArmAngleDegrees) {

        return m_robotUtilsFactory.createArmIoSim(
            Constants.SimIntakeArmConstants.kDeviceName,
            minArmAngleDegrees,
            maxArmAngleDegrees,
            Constants.SimIntakeArmConstants.kMoiKgM2,
            Constants.SimIntakeArmConstants.kArmLengthMeters,
            Constants.ArmConstants.kArmGearRatio);
    }
}
