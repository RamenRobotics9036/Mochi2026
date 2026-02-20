package frc.robot.subsystems.climber;

import frc.robot.sim.elevatorSim.ElevatorIoInterface;

/**
 * Real-hardware implementation of {@link ElevatorIoInterface} for the climber.
 *
 * <p>Methods are intentionally left blank for now; the real motor
 * control still lives in {@code ClimberSubsystem} until migration
 * is complete.
 */
public class ClimberIoReal implements ElevatorIoInterface {

    /** Constructor. */
    public ClimberIoReal() {
        // TODO: move hardware init from ClimberSubsystem here
    }

    @Override
    public void setSpeed(double speed) {
        // TODO: implement
    }

    @Override
    public void stop() {
        // TODO: implement
    }

    @Override
    public void updateOutputs(ElevatorIoInterface.DeviceOutputs outputs) {
        // TODO: implement
    }
}
