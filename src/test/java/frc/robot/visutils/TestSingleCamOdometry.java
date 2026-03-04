package frc.robot.visutils;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.sim.visionproducers.VisionSimInterface;
import java.util.List;
import org.junit.jupiter.api.*;
import org.mockito.MockedStatic;
import org.mockito.Mockito;

/**
 * Unit tests for {@link SingleCamOdometry}.
 *
 * <p>The class under test calls several {@link LimelightHelpers} static methods, so each test
 * runs with a {@link MockedStatic} wrapper opened in {@code @BeforeEach} and closed in
 * {@code @AfterEach}.  The constructor also calls {@code setCameraPose_RobotSpace}, which is a
 * void static — Mockito's default behaviour (do-nothing) is fine for it.
 *
 * <p>The key behaviours under test:
 * <ul>
 *   <li>No-target path — null / tagCount=0 clears all state.</li>
 *   <li>Single-tag std-dev heuristic — within 4 m accepted, beyond 4 m hard-rejected.</li>
 *   <li>Multi-tag std-dev heuristic — always accepted regardless of distance.</li>
 *   <li>Ambiguity cutoff at 0.7 — ambiguity > 0.7 → treated as no lock.</li>
 *   <li>Duplicate-timestamp guard — second call with same timestamp is skipped.</li>
 *   <li>Vision disabled — pose consumer is <em>not</em> called when vision is disabled.</li>
 *   <li>Kalman injection — only when motionless AND multi-tag.</li>
 *   <li>{@code getConfidenceScore()} exponential-decay mapping.</li>
 *   <li>Tag-ID / tx / visible-tag-list tracking from raw fiducials.</li>
 *   <li>State reset after lock loss.</li>
 * </ul>
 */
class TestSingleCamOdometry {

    private static final String CAM_NAME = "limelight-test";

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    // ------------------------------------------------------------------
    // Per-test state
    // ------------------------------------------------------------------

    /** Static mock kept open for the whole test so the constructor can also call statics. */
    private MockedStatic<LimelightHelpers> m_limelightMock;

    /** Captured pose/timestamp/stdDev calls from the estimate consumer. */
    private Pose2d m_lastConsumedPose;
    private int m_consumeCallCount;

    /** The instance under test. */
    private SingleCamOdometry m_cam;

    /** Mocked Kalman filter — lets us verify injection calls. */
    private VisionKalmanFilter m_kalmanFilter;

    @BeforeEach
    void setUp() {
        m_limelightMock = Mockito.mockStatic(LimelightHelpers.class);
        // getTX returns 0 by default unless overridden per-test.
        m_limelightMock.when(() -> LimelightHelpers.getTX(CAM_NAME)).thenReturn(0.0);

        m_lastConsumedPose = null;
        m_consumeCallCount = 0;

        VisionSimInterface.EstimateConsumer consumer =
            (pose, ts, stdDevs) -> {
                m_lastConsumedPose = pose;
                m_consumeCallCount++;
            };

        m_kalmanFilter = Mockito.mock(VisionKalmanFilter.class);

        // Identity transform — no rotation or translation from robot to camera.
        m_cam = new SingleCamOdometry(CAM_NAME, new Transform3d(), consumer);
    }

    @AfterEach
    void tearDown() {
        m_limelightMock.close();
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    /**
     * Builds a minimal single-tag {@link PoseEstimate} at a given average tag distance.
     * Ambiguity defaults to 0.0 (clean lock).
     */
    private static PoseEstimate singleTagEstimate(Pose2d pose, double avgTagDist,
                                                   double timestamp, double ambiguity) {
        RawFiducial fiducial = new RawFiducial(7, 0, 0, 0, avgTagDist, avgTagDist, ambiguity);
        return new PoseEstimate(pose, timestamp, 0.0,
            1, 0.0, avgTagDist, 0.0,
            new RawFiducial[]{fiducial}, false);
    }

    /** Builds a two-tag {@link PoseEstimate} at a given average tag distance. */
    private static PoseEstimate multiTagEstimate(Pose2d pose, double avgTagDist, double timestamp) {
        RawFiducial f1 = new RawFiducial(3, 0, 0, 0, avgTagDist, avgTagDist, 0.1);
        RawFiducial f2 = new RawFiducial(5, 0, 0, 0, avgTagDist, avgTagDist, 0.1);
        return new PoseEstimate(pose, timestamp, 0.0,
            2, 0.0, avgTagDist, 0.0,
            new RawFiducial[]{f1, f2}, false);
    }

    /** Representative pose used across several tests. */
    private static final Pose2d POSE_A = new Pose2d(3.0, 2.0, Rotation2d.kZero);

    // ------------------------------------------------------------------
    // 1. No-target path
    // ------------------------------------------------------------------

    /**
     * When {@code getBotPoseEstimate_wpiBlue} returns {@code null}, the camera must report
     * no target lock, an empty estimated pose, and a confidence score of 0.0.
     */
    @Test
    void periodic_nullEstimate_clearsAllState() {
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(null);

        m_cam.periodic();

        assertFalse(m_cam.hasTargetLock());
        assertTrue(m_cam.getEstimatedPose().isEmpty());
        assertEquals(0.0, m_cam.getConfidenceScore());
    }

    /**
     * When the pose estimate has {@code tagCount == 0} (2026 Limelight helpers no longer return
     * null when no targets are visible), the camera must treat it identically to a null estimate —
     * no lock, empty pose, score 0.
     */
    @Test
    void periodic_zeroTagCount_clearsAllState() {
        PoseEstimate zeroTags = new PoseEstimate(
            POSE_A, 1.0, 0.0, 0, 0.0, 0.0, 0.0, new RawFiducial[]{}, false);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(zeroTags);

        m_cam.periodic();

        assertFalse(m_cam.hasTargetLock());
        assertTrue(m_cam.getEstimatedPose().isEmpty());
        assertEquals(0.0, m_cam.getConfidenceScore());
    }

    /**
     * After a lock is established and then a cycle with no targets arrives, all state fields
     * ({@code hasTargetLock}, {@code getEstimatedPose}, {@code getConfidenceScore}) must return
     * to their cleared / default values.
     */
    @Test
    void periodic_lockThenNoTarget_clearsState() {
        // Establish a lock first.
        PoseEstimate est = singleTagEstimate(POSE_A, 2.0, 1.0, 0.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);
        m_cam.periodic();
        assertTrue(m_cam.hasTargetLock());

        // Next cycle: no targets.
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(null);
        m_cam.periodic();

        assertFalse(m_cam.hasTargetLock());
        assertTrue(m_cam.getEstimatedPose().isEmpty());
        assertEquals(0.0, m_cam.getConfidenceScore());
    }

    // ------------------------------------------------------------------
    // 2. Single-tag std-dev heuristic
    // ------------------------------------------------------------------

    /**
     * A single-tag estimate with average tag distance ≤ 4 m must be accepted:
     * {@code hasTargetLock()} returns {@code true} and the confidence score is positive.
     */
    @Test
    void periodic_singleTag_withinFourMeters_isAccepted() {
        PoseEstimate est = singleTagEstimate(POSE_A, 2.0, 1.0, 0.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        m_cam.periodic();

        assertTrue(m_cam.hasTargetLock());
        assertTrue(m_cam.getConfidenceScore() > 0.0);
    }

    /**
     * A single-tag estimate with average tag distance > 4 m must be hard-rejected:
     * the std devs are set to {@code Double.MAX_VALUE}, so the confidence score must be
     * 0.0 and {@code hasTargetLock()} must return {@code false}.
     */
    @Test
    void periodic_singleTag_beyondFourMeters_isRejected() {
        PoseEstimate est = singleTagEstimate(POSE_A, 5.0, 1.0, 0.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        m_cam.periodic();

        assertFalse(m_cam.hasTargetLock());
        assertEquals(0.0, m_cam.getConfidenceScore());
    }

    /**
     * A single-tag estimate at exactly 4 m (boundary) must also be rejected because the
     * rejection condition is {@code avgDist > 4}.  At exactly 4 m the estimate is still
     * within the accepted range and should be accepted with a positive confidence score.
     */
    @Test
    void periodic_singleTag_exactlyFourMeters_isAccepted() {
        PoseEstimate est = singleTagEstimate(POSE_A, 4.0, 1.0, 0.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        m_cam.periodic();

        assertTrue(m_cam.hasTargetLock());
        assertTrue(m_cam.getConfidenceScore() > 0.0);
    }

    // ------------------------------------------------------------------
    // 3. Multi-tag std-dev heuristic
    // ------------------------------------------------------------------

    /**
     * A multi-tag estimate at any distance must be accepted (the >4 m hard-rejection only
     * applies to single-tag estimates).  {@code hasTargetLock()}, {@code hasMultiTagLock()},
     * and a positive confidence score are expected.
     */
    @Test
    void periodic_multiTag_beyondFourMeters_isAccepted() {
        PoseEstimate est = multiTagEstimate(POSE_A, 6.0, 1.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        m_cam.periodic();

        assertTrue(m_cam.hasTargetLock());
        assertTrue(m_cam.hasMultiTagLock());
        assertTrue(m_cam.getConfidenceScore() > 0.0);
    }

    /**
     * For a multi-tag estimate the distance-scaling formula is applied:
     * {@code stdDevs * (1 + dist² / 30)}.  At greater distance the confidence score must
     * be lower than at shorter distance.
     */
    @Test
    void periodic_multiTag_greaterDistance_lowerConfidence() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    // ------------------------------------------------------------------
    // 4. Ambiguity cutoff
    // ------------------------------------------------------------------

    /**
     * A single-tag estimate whose sole raw fiducial has {@code ambiguity > 0.7} must be
     * rejected: {@code hasTargetLock()} returns {@code false} and confidence is 0.
     */
    @Test
    void periodic_singleTag_ambiguityAboveThreshold_isRejected() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    /**
     * A single-tag estimate with {@code ambiguity == 0.7} (exactly at the threshold) must
     * be accepted because the rejection condition is strictly {@code ambiguity > 0.7}.
     */
    @Test
    void periodic_singleTag_ambiguityExactlyAtThreshold_isAccepted() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    /**
     * The ambiguity check only applies when there is exactly one raw fiducial.  A
     * multi-tag estimate must be accepted even if one of the fiducials reports high ambiguity.
     */
    @Test
    void periodic_multiTag_highAmbiguity_isStillAccepted() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    // ------------------------------------------------------------------
    // 5. Duplicate-timestamp guard
    // ------------------------------------------------------------------

    /**
     * When {@code periodic()} is called twice with {@link PoseEstimate} objects that have
     * the same {@code timestampSeconds}, the second call must be a no-op: the pose consumer
     * must receive exactly one call and the visible-tag list must not be updated a second time.
     */
    @Test
    void periodic_duplicateTimestamp_secondCallIsIgnored() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    /**
     * Different timestamps on successive cycles must each be processed independently —
     * the consumer must receive two calls.
     */
    @Test
    void periodic_differentTimestamps_bothCallsAreProcessed() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    // ------------------------------------------------------------------
    // 6. Vision enabled / disabled
    // ------------------------------------------------------------------

    /**
     * When the vision-enabled supplier returns {@code false}, the estimate consumer must
     * <em>not</em> be called even though the pose estimate is valid.  The internal state
     * ({@code hasTargetLock}, {@code getEstimatedPose}) must still be updated.
     */
    @Test
    void periodic_visionDisabled_consumerNotCalled() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    /**
     * When the vision-enabled supplier returns {@code true} the consumer must be called
     * with the correct pose.
     */
    @Test
    void periodic_visionEnabled_consumerIsCalled() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    // ------------------------------------------------------------------
    // 7. Kalman filter injection
    // ------------------------------------------------------------------

    /**
     * When the robot is motionless AND the estimate has ≥ 2 tags, the Kalman filter's
     * {@code injectVisionMeasurement} must be called exactly once with the correct pose and
     * tag count.
     */
    @Test
    void periodic_motionlessAndMultiTag_injectsKalman() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    /**
     * When the robot is moving (motionless supplier returns {@code false}), the Kalman filter
     * must NOT be injected, even for a multi-tag estimate.
     */
    @Test
    void periodic_movingAndMultiTag_doesNotInjectKalman() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    /**
     * When the robot is motionless but the estimate has only a single tag, the Kalman filter
     * must NOT be injected.
     */
    @Test
    void periodic_motionlessAndSingleTag_doesNotInjectKalman() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    /**
     * When no Kalman filter has been wired via {@code setVisionDependencies}, calling
     * {@code periodic()} with a valid multi-tag estimate must not throw.
     */
    @Test
    void periodic_noKalmanFilterSet_doesNotThrow() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    // ------------------------------------------------------------------
    // 8. getConfidenceScore() exponential-decay mapping
    // ------------------------------------------------------------------

    /**
     * When std devs are set to {@code Double.MAX_VALUE} (rejected estimate), the confidence
     * score returned by {@code getConfidenceScore()} must be exactly {@code 0.0}.
     */
    @Test
    void getConfidenceScore_rejectedEstimate_returnsZero() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    /**
     * A near-zero-distance single-tag estimate produces a very small combined positional
     * uncertainty.  The resulting confidence score must be close to 100 (≥ 90).
     */
    @Test
    void getConfidenceScore_nearZeroDistance_returnsHighScore() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    /**
     * A multi-tag estimate at moderate distance produces a confidence score strictly between
     * 0 and the near-zero-distance score, confirming exponential decay with distance.
     */
    @Test
    void getConfidenceScore_multiTagModerateDistance_returnsIntermediateScore() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    // ------------------------------------------------------------------
    // 9. Tag-ID / tx / visible-tag-list tracking
    // ------------------------------------------------------------------

    /**
     * After a successful lock, {@code getVisibleTagIds()} must return a list that contains
     * exactly the IDs of the raw fiducials reported by the pose estimate, in the same order.
     */
    @Test
    void getVisibleTagIds_afterLock_returnsRawFiducialIds() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    /**
     * {@code getPrimaryTagId()} must return the ID of the first raw fiducial in the pose
     * estimate (index 0).
     */
    @Test
    void getPrimaryTagId_afterLock_returnsFirstFiducialId() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    /**
     * When no targets are visible, {@code getPrimaryTagId()} should return {@code -1}
     * (the "none" sentinel) and {@code getVisibleTagIds()} should return an empty list.
     */
    @Test
    void getVisibleTagIds_noTargets_returnsEmpty_andPrimaryTagIdIsMinusOne() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    /**
     * {@code getPrimaryTagTx()} must return the value from {@code LimelightHelpers.getTX()}
     * for the camera's name after a successful lock.
     */
    @Test
    void getPrimaryTagTx_afterLock_returnsLimelightTxValue() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    /**
     * {@code getPrimaryTagTx()} must return {@code 0.0} when no targets are visible (cleared
     * state).
     */
    @Test
    void getPrimaryTagTx_noTargets_returnsZero() {
        throw new UnsupportedOperationException("Not yet implemented");
    }
}
