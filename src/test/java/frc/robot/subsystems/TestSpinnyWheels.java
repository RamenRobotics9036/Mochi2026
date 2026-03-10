package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SpinnyWheelsConstants;
import robotutils.pub.interfaces.RollerIoInterface;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;


/**
 * Unit tests for {@link SpinnyWheels}.
 *
 * <p>All tests use a fake {@link RollerIoInterface} so no real hardware or
 * HAL simulation is required to run.
 */
class TestSpinnyWheels {

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    /** Minimal in-test fake that records the last speed set (0.0 after stop). */
    private static class FakeSpinnyIo implements RollerIoInterface {
        double m_lastSpeed = 0.0;
        int m_updateOutputsCallCount = 0;

        @Override
        public void setSpeed(double speed) {
            m_lastSpeed = speed;
        }

        @Override
        public void stop() {
            m_lastSpeed = 0.0;
        }

        @Override
        public void updateOutputs(DeviceOutputs outputs) {
            m_updateOutputsCallCount++;
            outputs.m_currentAmps = (m_lastSpeed != 0.0) ? 0.1 : 0.0;
            outputs.m_velocityRpm = (m_lastSpeed != 0.0) ? 100.0 : 0.0;
        }
    }

    private FakeSpinnyIo m_io;
    private SpinnyWheels m_spinny;

    @BeforeEach
    void setUp() {
        m_io = new FakeSpinnyIo();
        m_spinny = new SpinnyWheels(m_io);
    }

    // -----------------------------------------------------------------------
    // spin()
    // -----------------------------------------------------------------------

    @Test
    void spin_callsSetSpeedWithConstant() {
        // Calling spin() should forward the configured kSpinSpeed constant to the IO layer.
        m_spinny.spinCounterclockwise();
        assertEquals(SpinnyWheelsConstants.kSpinSpeed, m_io.m_lastSpeed, 1e-9);
    }

    @Test
    void spin_calledMultipleTimes_alwaysUsesConstantSpeed() {
        // Repeated calls to spin() should each forward the same kSpinSpeed value,
        // confirming no internal state drift occurs.
        for (int i = 0; i < 5; i++) {
            m_spinny.spinCounterclockwise();
            assertEquals(SpinnyWheelsConstants.kSpinSpeed, m_io.m_lastSpeed, 1e-9,
                "Speed should equal kSpinSpeed on call " + (i + 1));
        }
    }

    // -----------------------------------------------------------------------
    // stop()
    // -----------------------------------------------------------------------

    @Test
    void stop_afterSpin_succeeds() {
        // Calling spin() then stop() should leave the IO in the stopped state,
        // not the running state.
        m_spinny.spinCounterclockwise();
        m_spinny.stop();
        assertEquals(0.0, m_io.m_lastSpeed, 1e-9);
    }

    // -----------------------------------------------------------------------
    // periodic()
    // -----------------------------------------------------------------------

    @Test
    void periodic_callsUpdateOutputs() {
        // Each call to periodic() should invoke updateOutputs() on the IO layer
        // so sensor readings are refreshed every robot loop.
        m_spinny.periodic();
        assertEquals(1, m_io.m_updateOutputsCallCount);
        m_spinny.periodic();
        assertEquals(2, m_io.m_updateOutputsCallCount);
    }

    @Test
    void periodic_publishesVelocityToSmartDashboard() {
        // Reset NetworkTables value, since it may be 0.0 or NaN,
        // depending on the last test that ran.
        m_spinny.stop();
        m_spinny.periodic(); // publishes 0.0

        // periodic() should publish the velocity reading from the IO outputs to
        // SmartDashboard under the key "Spinny/VelocityRPM".
        m_spinny.spinCounterclockwise();

        // Before periodic()
        assertEquals(0.0, SmartDashboard.getNumber(
            "Spinny/VelocityRPM",
            Double.NaN), 1e-9);

        m_spinny.periodic();
        assertEquals(100.0, SmartDashboard.getNumber(
            "Spinny/VelocityRPM",
            Double.NaN), 1e-9);
    }
}
