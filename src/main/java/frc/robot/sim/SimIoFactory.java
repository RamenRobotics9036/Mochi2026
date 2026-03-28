package robotutils;

import frc.robot.Constants;
import frc.robot.sim.armsim.ArmIoInterface;
import frc.robot.sim.armsim.ArmIoSim;
import frc.robot.sim.elevatorssim.ElevatorIoInterface;
import frc.robot.sim.elevatorssim.ElevatorIoSim;
import frc.robot.sim.rollerssim.RollerIoInterface;
import frc.robot.sim.rollerssim.RollerIoSim;
import frc.robot.sim.rollerssim.TwoMotorRollerIoInterface;
import frc.robot.sim.rollerssim.TwoMotorRollerIoSim;

/** Helps create specific IO Sim instances for our specific robot. */
public class SimIoFactory {

    /**
     * Creates the shooter IO: a {@link TwoMotorRollerIoSim} in simulation.
     *
     * @return A new {@link TwoMotorRollerIoSim} configured with shooter sim constants.
     */
    public static TwoMotorRollerIoInterface createShooterIoSim() {
        return new TwoMotorRollerIoSim(
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
        return new RollerIoSim(
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
        return new RollerIoSim(
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
        return new ElevatorIoSim(
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
        return new RollerIoSim(
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

        return new ArmIoSim(
            Constants.SimIntakeArmConstants.kDeviceName,
            minArmAngleDegrees,
            maxArmAngleDegrees,
            Constants.SimIntakeArmConstants.kMoiKgM2,
            Constants.SimIntakeArmConstants.kArmLengthMeters,
            Constants.ArmConstants.kArmGearRatio);
    }
}
