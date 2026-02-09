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
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    private CommandSwerveDrivetrain mockDrive;
    private VisionKalmanFilter mockKalman;
    private DriveAccuracyTester tester;

    @BeforeEach
    void setUp() {
        mockDrive = mock(CommandSwerveDrivetrain.class);
        mockKalman = mock(VisionKalmanFilter.class);
        tester = new DriveAccuracyTester(mockDrive, mockKalman, b -> {});
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
        when(mockKalman.hasConverged()).thenReturn(false);

        Command cmd = tester.createTapeDropAutoCommand();

        // Capture System.out to verify the abort message
        PrintStream originalOut = System.out;
        ByteArrayOutputStream captured = new ByteArrayOutputStream();
        System.setOut(new PrintStream(captured));

        try {
            // Drive the command lifecycle directly so System.out capture works
            cmd.initialize();
            cmd.execute();
            cmd.end(cmd.isFinished());
        } finally {
            System.setOut(originalOut);
        }

        String output = captured.toString();
        assertTrue(
            output.contains("Kalman filter not converged \u2014 tape drop auto aborted."),
            "Expected abort message but got: " + output);

        // Blue tape should NOT have been placed
        assertTrue(tester.getBlueTapePose().isEmpty());
    }
}
