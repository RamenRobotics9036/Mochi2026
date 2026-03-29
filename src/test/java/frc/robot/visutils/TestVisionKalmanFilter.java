package frc.robot.visutils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.VisionKalmanConstants;
import robotutils.pub.interfaces.SimLimelightProducerInterface.DrivetrainVisionPoseInfo;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;


class TestVisionKalmanFilter {

    private static void inject(VisionKalmanFilter filter, Pose2d pose, int tagCount) {
        filter.accept(new DrivetrainVisionPoseInfo(pose, 0.0, null, tagCount));
    }

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    private VisionKalmanFilter m_filter;

    @BeforeEach
    void setUp() {
        m_filter = new VisionKalmanFilter();
    }

    // ---- Initial state ----

    @Test
    void test_initialState_notInitialized() {
        assertFalse(m_filter.isInitialized());
        assertFalse(m_filter.hasConverged());
        assertEquals(0, m_filter.getMeasurementCount());
        assertNull(m_filter.getCovariance());
    }

    @Test
    void test_initialState_estimateIsOrigin() {
        Pose2d estimate = m_filter.getEstimate();
        assertEquals(0.0, estimate.getX(), 1e-9);
        assertEquals(0.0, estimate.getY(), 1e-9);
        assertEquals(0.0, estimate.getRotation().getRadians(), 1e-9);
    }

    @Test
    void test_initialState_stdDevsAreMaxValue() {
        assertEquals(Double.MAX_VALUE, m_filter.getPositionStdDev());
        assertEquals(Double.MAX_VALUE, m_filter.getAngleStdDevDegrees());
    }

    // ---- Single-tag rejection ----

    @Test
    void test_singleTagMeasurement_rejected() {
        inject(m_filter,
            new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(30)), 1);

        assertFalse(m_filter.isInitialized(),
            "Single-tag measurement should be rejected");
        assertEquals(0, m_filter.getMeasurementCount());
    }

    @Test
    void test_zeroTagMeasurement_rejected() {
        inject(m_filter,
            new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(30)), 0);

        assertFalse(m_filter.isInitialized());
        assertEquals(0, m_filter.getMeasurementCount());
    }

    // ---- First measurement initialization ----

    @Test
    void test_firstMeasurement_initializesState() {
        Pose2d pose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(45));
        inject(m_filter, pose, 2);

        assertTrue(m_filter.isInitialized());
        assertEquals(1, m_filter.getMeasurementCount());

        Pose2d estimate = m_filter.getEstimate();
        assertEquals(2.0, estimate.getX(), 1e-9, "X should match first measurement");
        assertEquals(3.0, estimate.getY(), 1e-9, "Y should match first measurement");
        assertEquals(45.0, estimate.getRotation().getDegrees(), 1e-6, "Angle should match");
    }

    @Test
    void test_firstMeasurement_setsInitialCovariance() {
        inject(m_filter,
            new Pose2d(1.0, 1.0, new Rotation2d()), 2);

        var p = m_filter.getCovariance();
        assertNotNull(p);
        assertEquals(VisionKalmanConstants.kInitialPositionVariance, p.get(0, 0), 1e-9);
        assertEquals(VisionKalmanConstants.kInitialPositionVariance, p.get(1, 1), 1e-9);
        assertEquals(VisionKalmanConstants.kInitialAngleVariance, p.get(2, 2), 1e-9);
    }

    // ---- Convergence behavior ----

    @Test
    void test_estimateConverges_towardTruePose() {
        // Initialize with an intentionally wrong pose
        Pose2d wrongPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
        inject(m_filter, wrongPose, 2);
        double initialError = wrongPose.getTranslation().getDistance(
            new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)).getTranslation());

        // After one correction the estimate should move toward the true pose
        // but the covariance should NOT yet meet convergence thresholds
        Pose2d truePose = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90));
        inject(m_filter, truePose, 3);

        assertFalse(m_filter.hasConverged(),
            "Filter should not have converged after only 2 total measurements");

        // The estimate should have moved closer to the true pose than the initial guess
        Pose2d earlyEstimate = m_filter.getEstimate();
        double earlyError = earlyEstimate.getTranslation()
            .getDistance(truePose.getTranslation());
        assertTrue(earlyError < initialError,
            "Estimate should be closer to true pose after correction: "
            + "initialError=" + initialError + " earlyError=" + earlyError);

        // Now inject many more measurements at the true pose
        for (int i = 0; i < 50; i++) {
            inject(m_filter, truePose, 3);
        }

        // Final error should be smaller than early error
        Pose2d finalEstimate = m_filter.getEstimate();
        double finalError = finalEstimate.getTranslation()
            .getDistance(truePose.getTranslation());
        assertTrue(finalError < earlyError,
            "Final estimate should be closer than early: "
            + "earlyError=" + earlyError + " finalError=" + finalError);
        assertTrue(m_filter.hasConverged(),
            "Filter should have converged after many measurements");
    }

    @Test
    void test_hasConverged_falseInitially_trueAfterEnoughMeasurements() {
        Pose2d pose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(45));

        inject(m_filter, pose, 2);
        assertFalse(m_filter.hasConverged(),
            "Should not converge after just one measurement");

        // Inject until converged
        for (int i = 0; i < 200; i++) {
            inject(m_filter, pose, 3);
            if (m_filter.hasConverged()) {
                break;
            }
        }

        assertTrue(m_filter.hasConverged(), "Should converge after many measurements");
    }

    @Test
    void test_hasConvergedWithCustomThresholds() {
        Pose2d pose = new Pose2d(1.0, 1.0, new Rotation2d());

        inject(m_filter, pose, 2);

        // Very loose thresholds — should converge immediately
        assertTrue(m_filter.hasConverged(100.0, 100.0),
            "Should pass with very loose thresholds");

        // Very tight thresholds — should not converge
        assertFalse(m_filter.hasConverged(0.0001, 0.0001),
            "Should fail with very tight thresholds");
    }

    // ---- Reset ----

    @Test
    void test_reset_clearsAllState() {
        Pose2d pose = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90));
        for (int i = 0; i < 10; i++) {
            inject(m_filter, pose, 3);
        }
        assertTrue(m_filter.isInitialized());
        assertTrue(m_filter.getMeasurementCount() > 0);

        m_filter.reset();

        assertFalse(m_filter.isInitialized());
        assertEquals(0, m_filter.getMeasurementCount());
        assertNull(m_filter.getCovariance());
        assertEquals(Double.MAX_VALUE, m_filter.getPositionStdDev());
        assertEquals(Double.MAX_VALUE, m_filter.getAngleStdDevDegrees());
    }

    @Test
    void test_reset_allowsReinitializationAtNewPose() {
        inject(m_filter,
            new Pose2d(1.0, 1.0, new Rotation2d()), 2);
        m_filter.reset();

        Pose2d newPose = new Pose2d(8.0, 6.0, Rotation2d.fromDegrees(180));
        inject(m_filter, newPose, 2);

        assertTrue(m_filter.isInitialized());
        assertEquals(1, m_filter.getMeasurementCount());
        assertEquals(8.0, m_filter.getEstimate().getX(), 1e-9);
        assertEquals(6.0, m_filter.getEstimate().getY(), 1e-9);
    }

    // ---- Angle wrapping ----

    @Test
    void test_angleWrapping_nearPositive180() {
        // Initialize at +170°, then inject measurements at +175°
        inject(m_filter,
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(170)), 2);

        for (int i = 0; i < 50; i++) {
            inject(m_filter,
                new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(175)), 3);
        }

        double angularError = m_filter.getEstimate().getRotation()
            .minus(Rotation2d.fromDegrees(175)).getDegrees();
        assertTrue(Math.abs(angularError) < 1.0,
            "Angle should converge near +175°, angular error: " + angularError);
    }

    @Test
    void test_angleWrapping_acrossBoundary() {
        // Initialize at +170°, then inject measurements at -170° (across the ±180° boundary)
        inject(m_filter,
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(170)), 2);

        for (int i = 0; i < 50; i++) {
            inject(m_filter,
                new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(-170)), 3);
        }

        // Should converge near -170°, NOT jump to ~0° (which would happen without wrapping)
        double angularError = m_filter.getEstimate().getRotation()
            .minus(Rotation2d.fromDegrees(-170)).getDegrees();
        assertTrue(Math.abs(angularError) < 5.0,
            "Angle should converge near -170°, angular error: " + angularError);
    }

    @Test
    void test_angleWrapping_nearNegative180() {
        inject(m_filter,
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(-175)), 2);

        for (int i = 0; i < 50; i++) {
            inject(m_filter,
                new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(-170)), 3);
        }

        double angularError = m_filter.getEstimate().getRotation()
            .minus(Rotation2d.fromDegrees(-170)).getDegrees();
        assertTrue(Math.abs(angularError) < 1.0,
            "Angle should converge near -170°, angular error: " + angularError);
    }

    // ---- Std dev decreases with more measurements ----

    @Test
    void test_positionStdDev_decreasesWithMeasurements() {
        Pose2d pose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(0));

        inject(m_filter, pose, 2);
        double stdDevAfter1 = m_filter.getPositionStdDev();

        for (int i = 0; i < 10; i++) {
            inject(m_filter, pose, 2);
        }
        double stdDevAfter11 = m_filter.getPositionStdDev();

        for (int i = 0; i < 40; i++) {
            inject(m_filter, pose, 2);
        }
        double stdDevAfter51 = m_filter.getPositionStdDev();

        assertTrue(stdDevAfter11 < stdDevAfter1,
            "Std dev should decrease: after1=" + stdDevAfter1 + " after11=" + stdDevAfter11);
        assertTrue(stdDevAfter51 < stdDevAfter11,
            "Std dev should decrease: after11=" + stdDevAfter11 + " after51=" + stdDevAfter51);
    }

    @Test
    void test_angleStdDevDegrees_decreasesWithMeasurements() {
        Pose2d pose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(45));

        inject(m_filter, pose, 2);
        double angleStdAfter1 = m_filter.getAngleStdDevDegrees();

        for (int i = 0; i < 20; i++) {
            inject(m_filter, pose, 3);
        }
        double angleStdAfter21 = m_filter.getAngleStdDevDegrees();

        assertTrue(angleStdAfter21 < angleStdAfter1,
            "Angle std dev should decrease: after1=" + angleStdAfter1
            + " after21=" + angleStdAfter21);
    }

    // ---- getFieldDisplayInfo ----

    @Test
    void test_getFieldDisplayInfo_emptyWhenUninitialized() {
        var info = m_filter.getFieldDisplayInfo(0.5);

        assertTrue(info.pose().isEmpty(), "Pose should be empty when uninitialized");
        assertFalse(info.hasConverged());
    }

    @Test
    void test_getFieldDisplayInfo_returnsOffsetPoseWhenInitialized() {
        Pose2d pose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(0));
        inject(m_filter, pose, 2);

        double offset = 0.5;
        var info = m_filter.getFieldDisplayInfo(offset);

        assertTrue(info.pose().isPresent(), "Pose should be present when initialized");

        Pose2d displayPose = info.pose().get();
        // At 0° heading, forward offset adds to X
        assertEquals(3.0 + offset, displayPose.getX(), 0.01,
            "X should be offset forward");
        assertEquals(4.0, displayPose.getY(), 0.01,
            "Y should stay the same at 0° heading");
    }

    @Test
    void test_getFieldDisplayInfo_offsetRotatesWithHeading() {
        // At 90° heading, forward offset adds to Y instead of X
        Pose2d pose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(90));
        inject(m_filter, pose, 2);

        double offset = 0.5;
        var info = m_filter.getFieldDisplayInfo(offset);
        Pose2d displayPose = info.pose().get();

        assertEquals(3.0, displayPose.getX(), 0.01,
            "X should stay ~same at 90° heading");
        assertEquals(4.0 + offset, displayPose.getY(), 0.01,
            "Y should be offset forward at 90° heading");
    }

    @Test
    void test_getFieldDisplayInfo_convergenceStatusReflected() {
        Pose2d pose = new Pose2d(1.0, 1.0, new Rotation2d());

        inject(m_filter, pose, 2);
        assertFalse(m_filter.getFieldDisplayInfo(0).hasConverged(),
            "Should not be converged after one measurement");

        for (int i = 0; i < 200; i++) {
            inject(m_filter, pose, 4);
        }
        assertTrue(m_filter.getFieldDisplayInfo(0).hasConverged(),
            "Should be converged after many measurements");
    }

    // ---- More tags = faster convergence ----

    @Test
    void test_moreTagsMeansFasterConvergence() {
        Pose2d pose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(0));
        int numMeasurements = 20;

        // Filter with 2 tags
        VisionKalmanFilter filter2Tags = new VisionKalmanFilter();
        for (int i = 0; i < numMeasurements; i++) {
            inject(filter2Tags, pose, 2);
        }

        // Filter with 4 tags
        VisionKalmanFilter filter4Tags = new VisionKalmanFilter();
        for (int i = 0; i < numMeasurements; i++) {
            inject(filter4Tags, pose, 4);
        }

        double stdDev2Tags = filter2Tags.getPositionStdDev();
        double stdDev4Tags = filter4Tags.getPositionStdDev();

        assertTrue(stdDev4Tags < stdDev2Tags,
            "4 tags should converge faster (lower std dev) than 2 tags: "
            + "2tags=" + stdDev2Tags + " 4tags=" + stdDev4Tags);
    }

    // ---- Measurement count ----

    @Test
    void test_measurementCount_incrementsCorrectly() {
        Pose2d pose = new Pose2d(1.0, 1.0, new Rotation2d());

        // Single-tag should not increment
        inject(m_filter, pose, 1);
        assertEquals(0, m_filter.getMeasurementCount());

        // Multi-tag should increment
        inject(m_filter, pose, 2);
        assertEquals(1, m_filter.getMeasurementCount());

        inject(m_filter, pose, 3);
        assertEquals(2, m_filter.getMeasurementCount());

        inject(m_filter, pose, 4);
        assertEquals(3, m_filter.getMeasurementCount());
    }
}
