package frc.robot.visutils;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import org.junit.jupiter.api.*;

class DriveAccuracyTesterTest {

    @BeforeAll
    static void initHal() {
        HAL.initialize(500, 0);
    }

    private CommandSwerveDrivetrain m_mockDrive;
    private VisionKalmanFilter m_mockKalman;
    private DriveAccuracyTester m_tester;

    @BeforeEach
    void setUp() {
        m_mockDrive = mock(CommandSwerveDrivetrain.class);
        m_mockKalman = mock(VisionKalmanFilter.class);
        m_tester = new DriveAccuracyTester(m_mockDrive, m_mockKalman, b -> {});
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    @Test
    void createTapeDropAutoCommand_abortsWhenKalmanNotConverged() {
        // Setup mock return-values
        when(m_mockKalman.hasConverged()).thenReturn(false);

        Command cmd = m_tester.createTapeDropAutoCommand();

        String output = runAndCaptureOutput(() -> {
            cmd.initialize();
            cmd.execute();
            cmd.end(cmd.isFinished());
        });

        assertTrue(
            output.contains("Kalman filter not converged \u2014 tape drop auto aborted."),
            "Expected abort message but got: " + output);

        // Blue tape should NOT have been placed
        assertTrue(m_tester.getBlueTapePose().isEmpty());
    }

    /**
     * Runs the given action while capturing everything written to System.out,
     * then restores the original stream and returns the captured text.
     */
    private String runAndCaptureOutput(Runnable action) {
        PrintStream originalOut = System.out;
        ByteArrayOutputStream captured = new ByteArrayOutputStream();
        System.setOut(new PrintStream(captured));
        try {
            action.run();
        } finally {
            System.setOut(originalOut);
        }
        return captured.toString();
    }
}
