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
import java.util.function.Consumer;
import org.junit.jupiter.api.*;
import org.mockito.MockedStatic;

class TestDriveAccuracyTester {

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
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
        runCommandToCompletion(cmd, maxSteps, () -> {});
    }

    /**
     * Initializes and repeatedly executes a command until it finishes or the
     * step limit is reached. Calls end() when done.
     *
     * @param cmd The command to run
     * @param maxSteps Maximum number of execute() cycles before giving up
     * @param afterInitialize Callback invoked right after cmd.initialize(), before
     *                        the execute loop begins — useful for mid-lifecycle assertions
     * @throws IllegalStateException if the command did not finish within maxSteps
     */
    private void runCommandToCompletion(Command cmd, int maxSteps, Runnable afterInitialize) {
        SimHooks.pauseTiming();
        try {
            cmd.initialize();
            afterInitialize.run();
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
     * Initializes and runs a command for exactly {@code steps} cycles, then
     * calls end(true) to simulate a scheduler cancellation mid-execution.
     *
     * @param cmd   The command to run
     * @param steps Number of execute() cycles before cancelling
     */
    private void runCommandThenCancel(Command cmd, int steps) {
        SimHooks.pauseTiming();
        try {
            cmd.initialize();
            for (int i = 0; i < steps; i++) {
                SimHooks.stepTiming(0.02);
                cmd.execute();
            }
            cmd.end(true);
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

            // Verify blue tape position matches expected front-of-robot offset
            Pose2d expectedTapePose = m_tester.computeFrontTapePose(kalmanPose);
            Pose2d blueTape = m_tester.getBlueTapePose().get();
            assertEquals(expectedTapePose.getX(), blueTape.getX(), 1e-6, "Blue tape X");
            assertEquals(expectedTapePose.getY(), blueTape.getY(), 1e-6, "Blue tape Y");
            assertEquals(expectedTapePose.getRotation().getDegrees(), blueTape.getRotation().getDegrees(), 1e-6, "Blue tape rotation");

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
            // Note: We only check the LAST line of output since theres other text too
            assertTrue(output.contains("Failed to lock onto camera at end of cycle."),
                "Expected camera lock failure message but got: " + output);
        }
    }

    @Test
    void test_forceDisableVision_calledCorrectly() {
        @SuppressWarnings("unchecked")
        Consumer<Boolean> mockForceDisableVision = mock(Consumer.class);

        DriveAccuracyTester tester = new DriveAccuracyTester(
            m_mockDrive, m_mockKalman, mockForceDisableVision);

        when(m_mockKalman.hasConverged()).thenReturn(true);
        when(m_mockKalman.getEstimate())
            .thenReturn(new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(45)));

        try (MockedStatic<AutoLogic> mockedAutoLogic = mockStatic(AutoLogic.class)) {
            mockedAutoLogic.when(AutoLogic::getSelectedAutoStartingPose)
                .thenReturn(Pose2d.kZero);
            mockedAutoLogic.when(AutoLogic::getSelectedAutoCommand)
                .thenReturn(Commands.none());

            Command cmd = tester.createTapeDropAutoCommand();

            // After creation, forceDisableVision should NOT have been called
            verify(mockForceDisableVision, never()).accept(any());

            runCommandToCompletion(cmd, 100, () -> {
                // After initialize, vision should be disabled (true)
                verify(mockForceDisableVision).accept(true);
                verify(mockForceDisableVision, never()).accept(false);
            });

            // After end, vision should be re-enabled (false)
            verify(mockForceDisableVision).accept(false);
        }
    }

    @Test
    void test_clearTape_clearsBothPoses() {
        // First, run a happy path to place both tapes
        when(m_mockKalman.hasConverged()).thenReturn(true);
        when(m_mockKalman.getEstimate())
            .thenReturn(new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(45)));

        try (MockedStatic<AutoLogic> mockedAutoLogic = mockStatic(AutoLogic.class)) {
            mockedAutoLogic.when(AutoLogic::getSelectedAutoStartingPose)
                .thenReturn(Pose2d.kZero);
            mockedAutoLogic.when(AutoLogic::getSelectedAutoCommand)
                .thenReturn(Commands.none());

            Command cmd = m_tester.createTapeDropAutoCommand();
            runCommandToCompletion(cmd, 100);
        }

        // Both tapes should be present after happy path
        assertTrue(m_tester.getBlueTapePose().isPresent(), "Blue tape should exist before clear");
        assertTrue(m_tester.getRedTapePose().isPresent(), "Red tape should exist before clear");

        // Clear and verify
        m_tester.clearTape();
        assertTrue(m_tester.getBlueTapePose().isEmpty(), "Blue tape should be empty after clear");
        assertTrue(m_tester.getRedTapePose().isEmpty(), "Red tape should be empty after clear");
    }

    @Test
    void test_preExistingTape_clearedAtSequenceStart() {
        Pose2d firstPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(0));
        Pose2d secondPose = new Pose2d(5.0, 6.0, Rotation2d.fromDegrees(90));

        when(m_mockKalman.hasConverged()).thenReturn(true);

        try (MockedStatic<AutoLogic> mockedAutoLogic = mockStatic(AutoLogic.class)) {
            mockedAutoLogic.when(AutoLogic::getSelectedAutoStartingPose)
                .thenReturn(Pose2d.kZero);
            mockedAutoLogic.when(AutoLogic::getSelectedAutoCommand)
                .thenAnswer(invocation -> Commands.none());

            // First run — places tape at firstPose
            when(m_mockKalman.getEstimate()).thenReturn(firstPose);
            Command cmd1 = m_tester.createTapeDropAutoCommand();
            runCommandToCompletion(cmd1, 100);

            Pose2d firstBlueTape = m_tester.getBlueTapePose().get();
            Pose2d firstRedTape = m_tester.getRedTapePose().get();

            // Second run with a different Kalman estimate
            when(m_mockKalman.getEstimate()).thenReturn(secondPose);
            Command cmd2 = m_tester.createTapeDropAutoCommand();
            runCommandToCompletion(cmd2, 100);

            // Verify tape positions changed — old tape was cleared, new tape placed
            Pose2d expectedNewBlue = m_tester.computeFrontTapePose(secondPose);
            Pose2d newBlueTape = m_tester.getBlueTapePose().get();

            assertNotEquals(firstBlueTape.getX(), newBlueTape.getX(), 1e-6,
                "Blue tape X should differ between runs");
            assertEquals(expectedNewBlue.getX(), newBlueTape.getX(), 1e-6,
                "Blue tape X should match second estimate");
            assertEquals(expectedNewBlue.getY(), newBlueTape.getY(), 1e-6,
                "Blue tape Y should match second estimate");

            Pose2d expectedNewRed = m_tester.computeFrontTapePose(secondPose);
            Pose2d newRedTape = m_tester.getRedTapePose().get();
            assertNotEquals(firstRedTape.getX(), newRedTape.getX(), 1e-6,
                "Red tape X should differ between runs");
            assertEquals(expectedNewRed.getX(), newRedTape.getX(), 1e-6,
                "Red tape X should match second estimate");
        }
    }

    @Test
    void test_forceDisableVision_reenabledWhenPreconditionsFail() {
        @SuppressWarnings("unchecked")
        Consumer<Boolean> mockForceDisableVision = mock(Consumer.class);

        DriveAccuracyTester tester = new DriveAccuracyTester(
            m_mockDrive, m_mockKalman, mockForceDisableVision);

        // Kalman NOT converged — preconditions will fail
        when(m_mockKalman.hasConverged()).thenReturn(false);

        Command cmd = tester.createTapeDropAutoCommand();

        runAndCaptureOutput(() -> {
            runCommandToCompletion(cmd, 100);
        });

        // Even though preconditions failed, vision should have been
        // disabled (beforeStarting) then re-enabled (finallyDo)
        var inOrder = inOrder(mockForceDisableVision);
        inOrder.verify(mockForceDisableVision).accept(true);
        inOrder.verify(mockForceDisableVision).accept(false);
    }

    @Test
    void test_cancelledMidWay_reenablesVision() {
        @SuppressWarnings("unchecked")
        Consumer<Boolean> mockForceDisableVision = mock(Consumer.class);

        DriveAccuracyTester tester = new DriveAccuracyTester(
            m_mockDrive, m_mockKalman, mockForceDisableVision);

        when(m_mockKalman.hasConverged()).thenReturn(true);
        when(m_mockKalman.getEstimate())
            .thenReturn(new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(45)));

        try (MockedStatic<AutoLogic> mockedAutoLogic = mockStatic(AutoLogic.class)) {
            mockedAutoLogic.when(AutoLogic::getSelectedAutoStartingPose)
                .thenReturn(Pose2d.kZero);
            // Use a long-running auto so we can cancel mid-way
            mockedAutoLogic.when(AutoLogic::getSelectedAutoCommand)
                .thenReturn(Commands.waitSeconds(100));

            Command cmd = tester.createTapeDropAutoCommand();

            // Run 100 cycles (enough to get past clearTape/dropBlueTape,
            // into the long-running auto) then simulate scheduler cancellation
            runCommandThenCancel(cmd, 100);

            // Vision should have been disabled then re-enabled despite cancellation
            var inOrder = inOrder(mockForceDisableVision);
            inOrder.verify(mockForceDisableVision).accept(true);
            inOrder.verify(mockForceDisableVision).accept(false);
        }
    }

    @Test
    void test_commandCanBeReusedTwice() {
        when(m_mockKalman.hasConverged()).thenReturn(true);
        Pose2d firstPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(0));
        Pose2d secondPose = new Pose2d(5.0, 6.0, Rotation2d.fromDegrees(90));

        try (MockedStatic<AutoLogic> mockedAutoLogic = mockStatic(AutoLogic.class)) {
            mockedAutoLogic.when(AutoLogic::getSelectedAutoStartingPose)
                .thenReturn(Pose2d.kZero);
            mockedAutoLogic.when(AutoLogic::getSelectedAutoCommand)
                .thenReturn(Commands.none());

            // Create ONE command instance
            when(m_mockKalman.getEstimate()).thenReturn(firstPose);
            Command cmd = m_tester.createTapeDropAutoCommand();

            // First run
            runCommandToCompletion(cmd, 100);
            assertTrue(m_tester.getBlueTapePose().isPresent(), "First run: blue tape");
            assertTrue(m_tester.getRedTapePose().isPresent(), "First run: red tape");
            Pose2d firstBlue = m_tester.getBlueTapePose().get();

            // Second run — SAME Command instance, different Kalman estimate
            when(m_mockKalman.getEstimate()).thenReturn(secondPose);
            runCommandToCompletion(cmd, 100);

            assertTrue(m_tester.getBlueTapePose().isPresent(), "Second run: blue tape");
            assertTrue(m_tester.getRedTapePose().isPresent(), "Second run: red tape");

            Pose2d secondBlue = m_tester.getBlueTapePose().get();
            Pose2d expectedSecondBlue = m_tester.computeFrontTapePose(secondPose);

            assertEquals(expectedSecondBlue.getX(), secondBlue.getX(), 1e-6,
                "Second run blue tape should use new estimate");
            assertNotEquals(firstBlue.getX(), secondBlue.getX(), 1e-6,
                "Tape positions should differ between runs");
        }
    }
}
