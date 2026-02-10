package frc.robot.visutils;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.auto.AutoLogic;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import org.junit.jupiter.api.*;
import org.mockito.MockedStatic;

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

    /**
     * Initializes and repeatedly executes a command until it finishes or the
     * step limit is reached. Calls end() when done.
     *
     * @param cmd The command to run
     * @param maxSteps Maximum number of execute() cycles before giving up
     * @throws IllegalStateException if the command did not finish within maxSteps
     */
    private void runCommandToCompletion(Command cmd, int maxSteps) {
        SimHooks.pauseTiming();
        try {
            cmd.initialize();
            int steps = 0;
            while (!cmd.isFinished() && steps < maxSteps) {
                SimHooks.stepTiming(0.02); // Advance simulated clock by one 20ms cycle
                cmd.execute();
                steps++;
            }

            if (!cmd.isFinished()) {
                throw new IllegalStateException(
                    "Command did not finish within " + maxSteps + " steps");
            }

            cmd.end(false);
        } finally {
            SimHooks.resumeTiming();
        }
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

    @Test
    void test_createTapeDropAutoCommand_abortsWhenKalmanNotConverged() {
        // Setup mock return-values
        when(m_mockKalman.hasConverged()).thenReturn(false);

        Command cmd = m_tester.createTapeDropAutoCommand();

        String output = runAndCaptureOutput(() -> {
            runCommandToCompletion(cmd, 100);
        });

        assertEquals(
            "Kalman filter not converged, tape drop auto aborted.",
            output.trim());

        // Blue tape should NOT have been placed
        assertTrue(m_tester.getBlueTapePose().isEmpty());
    }

    @Test
    void test_createTapeDropAutoCommand_abortsWhenAutoHasStartPose() {
        // Kalman converged — passes first check
        when(m_mockKalman.hasConverged()).thenReturn(true);

        try (MockedStatic<AutoLogic> mockedAutoLogic = mockStatic(AutoLogic.class)) {
            // Auto has a non-zero starting pose — should fail second check
            mockedAutoLogic.when(AutoLogic::getSelectedAutoStartingPose)
                .thenReturn(new Pose2d(1, 2, new Rotation2d()));
            mockedAutoLogic.when(AutoLogic::getSelectedAutoCommand)
                .thenReturn(Commands.none());

            Command cmd = m_tester.createTapeDropAutoCommand();

            String output = runAndCaptureOutput(() -> {
                runCommandToCompletion(cmd, 100);
            });

            assertEquals(
                "Only relative auto paths are supported.",
                output.trim());
        }

        // Blue tape should NOT have been placed
        assertTrue(m_tester.getBlueTapePose().isEmpty());
    }

    @Test
    void test_happyDayCase_dropsBlueTape() {
        // Kalman converged and returns a known pose
        when(m_mockKalman.hasConverged()).thenReturn(true);
        Pose2d kalmanPose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(45));
        when(m_mockKalman.getEstimate()).thenReturn(kalmanPose);

        try (MockedStatic<AutoLogic> mockedAutoLogic = mockStatic(AutoLogic.class)) {
            mockedAutoLogic.when(AutoLogic::getSelectedAutoStartingPose)
                .thenReturn(Pose2d.kZero);
            mockedAutoLogic.when(AutoLogic::getSelectedAutoCommand)
                .thenReturn(Commands.none());

            Command cmd = m_tester.createTapeDropAutoCommand();

            runCommandToCompletion(cmd, 100);

            assertTrue(m_tester.getBlueTapePose().isPresent(),
                "Blue tape should have been placed");

            assertTrue(m_tester.getRedTapePose().isPresent(),
                "Red tape should have been placed");
        }
    }

    @Test
    void test_kalmanDoesntConvergeAtEndOfTest() {
        // First call returns true (passes precondition), then false thereafter
        when(m_mockKalman.hasConverged()).thenReturn(true, false);
        Pose2d kalmanPose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(45));
        when(m_mockKalman.getEstimate()).thenReturn(kalmanPose);

        try (MockedStatic<AutoLogic> mockedAutoLogic = mockStatic(AutoLogic.class)) {
            mockedAutoLogic.when(AutoLogic::getSelectedAutoStartingPose)
                .thenReturn(Pose2d.kZero);
            mockedAutoLogic.when(AutoLogic::getSelectedAutoCommand)
                .thenReturn(Commands.none());

            Command cmd = m_tester.createTapeDropAutoCommand();

            String output = runAndCaptureOutput(() -> {
                // We need to run a lot of cycles to pass 10s
                runCommandToCompletion(cmd, 1000);
            });

            // Blue tape should still be placed (happens before the auto)
            assertTrue(m_tester.getBlueTapePose().isPresent(),
                "Blue tape should have been placed");

            // Red tape should NOT be placed (Kalman never re-converged)
            assertTrue(m_tester.getRedTapePose().isEmpty(),
                "Red tape should not have been placed");

            // Should see the failure message
            assertTrue(output.contains("Failed to lock onto camera at end of cycle."),
                "Expected camera lock failure message but got: " + output);
        }
    }
}
