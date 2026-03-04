package frc.robot.visutils;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.*;
import org.mockito.Mockito;

/**
 * Unit tests for {@link MultiCamOdometry}.
 *
 * <p>All {@link CamOdometryInterface} dependencies are Mockito mocks so tests
 * focus purely on the aggregation logic inside {@code MultiCamOdometry} without
 * touching any real hardware or Limelight code.
 *
 * <p>The key behaviors under test:
 * <ul>
 *   <li>Dependency wiring – {@code setVisionDependencies} is forwarded to every camera.</li>
 *   <li>Periodic fan-out – {@code periodic()} drives every camera.</li>
 *   <li>Best-camera selection – highest {@code getConfidenceScore()} wins; score ≤ 0 is
 *       never selected.</li>
 *   <li>Pose/score accessors delegate to the winning camera.</li>
 *   <li>Boolean aggregation – {@code hasTargetLock}/{@code hasMultiTagLock} are OR across cams.</li>
 *   <li>Tag-ID union – {@code getVisibleTagIds} merges, deduplicates, and sorts.</li>
 *   <li>Primary-camera pin – tx and primary-tag ID always come from camera 0.</li>
 *   <li>Empty-camera guard – IllegalStateException when no cameras are present.</li>
 * </ul>
 */
class TestMultiCamOdometry {

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    // ------------------------------------------------------------------
    // Shared test fixtures
    // ------------------------------------------------------------------

    /** A representative pose near the centre of a typical FRC field. */
    private static final Pose2d POSE_A = new Pose2d(3.0, 2.0, Rotation2d.kZero);

    /** A second distinct pose for multi-camera tests. */
    private static final Pose2d POSE_B = new Pose2d(5.0, 4.0, Rotation2d.kZero);

    /** Convenience: create a fresh mock that boots with sensible "no lock" defaults. */
    private static CamOdometryInterface newMockCam() {
        CamOdometryInterface mock = Mockito.mock(CamOdometryInterface.class);
        Mockito.when(mock.getConfidenceScore()).thenReturn(0.0);
        Mockito.when(mock.hasTargetLock()).thenReturn(false);
        Mockito.when(mock.hasMultiTagLock()).thenReturn(false);
        Mockito.when(mock.getEstimatedPose()).thenReturn(Optional.empty());
        Mockito.when(mock.getVisibleTagIds()).thenReturn(List.of());
        Mockito.when(mock.getPrimaryTagId()).thenReturn(-1);
        Mockito.when(mock.getPrimaryTagTx()).thenReturn(0.0);
        return mock;
    }

    // ------------------------------------------------------------------
    // 1. setVisionDependencies — wiring propagation
    // ------------------------------------------------------------------

    /**
     * {@code setVisionDependencies} must be forwarded to every camera in the list;
     * the same supplier and filter references should reach cam0 and cam1.
     */
    @Test
    void setVisionDependencies_propagatesToAllCameras() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        BooleanSupplier visionEnabled = () -> true;
        VisionKalmanFilter filter = Mockito.mock(VisionKalmanFilter.class);
        BooleanSupplier isMotionless = () -> false;

        multiCam.setVisionDependencies(visionEnabled, filter, isMotionless);

        // Verify that both cameras received the exact same dependency references.
        Mockito.verify(cam0).setVisionDependencies(visionEnabled, filter, isMotionless);
        Mockito.verify(cam1).setVisionDependencies(visionEnabled, filter, isMotionless);
    }

    // ------------------------------------------------------------------
    // 2. periodic() — fan-out
    // ------------------------------------------------------------------

    /**
     * Each call to {@code MultiCamOdometry.periodic()} must invoke {@code periodic()}
     * on every registered camera exactly once.
     */
    @Test
    void periodic_callsPeriodicOnAllCameras() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        multiCam.periodic();

        // Each camera must have had periodic() called exactly once.
        Mockito.verify(cam0, Mockito.times(1)).periodic();
        Mockito.verify(cam1, Mockito.times(1)).periodic();
    }

    /**
     * On the second call to {@code periodic()}, per-cycle state must be reset so
     * that data from the previous cycle does not bleed into the new one.  In
     * particular, if cam0 had a lock in cycle N but not in cycle N+1, the
     * accessors must reflect the absence of a lock after cycle N+1.
     */
    @Test
    void periodic_stateResetsEachCycle() {
        CamOdometryInterface cam0 = newMockCam();
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0));

        // Cycle N: cam0 has a strong lock.
        Mockito.when(cam0.getConfidenceScore()).thenReturn(5.0);
        Mockito.when(cam0.hasTargetLock()).thenReturn(true);
        Mockito.when(cam0.getEstimatedPose()).thenReturn(Optional.of(POSE_A));
        multiCam.periodic();
        assertTrue(multiCam.hasTargetLock());
        assertTrue(multiCam.getEstimatedPose().isPresent());

        // Cycle N+1: cam0 has lost lock; stale state must not carry over.
        Mockito.when(cam0.getConfidenceScore()).thenReturn(0.0);
        Mockito.when(cam0.hasTargetLock()).thenReturn(false);
        Mockito.when(cam0.getEstimatedPose()).thenReturn(Optional.empty());
        multiCam.periodic();

        assertFalse(multiCam.hasTargetLock(), "hasTargetLock should be false after lock is lost");
        assertTrue(multiCam.getEstimatedPose().isEmpty(), "getEstimatedPose should be empty after lock is lost");
    }

    // ------------------------------------------------------------------
    // 3. getEstimatedPose()
    // ------------------------------------------------------------------

    /**
     * When no camera has a positive confidence score after {@code periodic()},
     * {@code getEstimatedPose()} returns {@code Optional.empty()}.
     */
    @Test
    void getEstimatedPose_noCamsHaveLock_returnsEmpty() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        // Both mocks default to score 0.0 — no lock.
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        multiCam.periodic();

        assertTrue(multiCam.getEstimatedPose().isEmpty());
    }

    /**
     * With a single camera that has a positive confidence score,
     * {@code getEstimatedPose()} returns that camera's pose.
     */
    @Test
    void getEstimatedPose_singleCamWithLock_returnsCamPose() {
        CamOdometryInterface cam0 = newMockCam();
        Mockito.when(cam0.getConfidenceScore()).thenReturn(3.0);
        Mockito.when(cam0.getEstimatedPose()).thenReturn(Optional.of(POSE_A));
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0));

        multiCam.periodic();

        assertEquals(Optional.of(POSE_A), multiCam.getEstimatedPose());
    }

    /**
     * When two cameras both have positive confidence scores, the pose from the
     * camera with the <em>higher</em> score is returned.
     */
    @Test
    void getEstimatedPose_twoCamsLocked_returnsHigherScoredCamPose() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        // cam0 has a weaker lock; cam1 has a stronger one.
        Mockito.when(cam0.getConfidenceScore()).thenReturn(2.0);
        Mockito.when(cam0.getEstimatedPose()).thenReturn(Optional.of(POSE_A));
        Mockito.when(cam1.getConfidenceScore()).thenReturn(5.0);
        Mockito.when(cam1.getEstimatedPose()).thenReturn(Optional.of(POSE_B));
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        multiCam.periodic();

        // POSE_B belongs to cam1 which has the higher score.
        assertEquals(Optional.of(POSE_B), multiCam.getEstimatedPose());
    }

    /**
     * A camera whose {@code getConfidenceScore()} returns exactly 0.0 (i.e. no
     * active lock) must not be selected — {@code getEstimatedPose()} still returns
     * empty when all scores are ≤ 0.
     */
    @Test
    void getEstimatedPose_camsWithZeroScore_returnsEmpty() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        // Both cams have score exactly 0.0 — the guard (score > 0) must exclude them.
        Mockito.when(cam0.getConfidenceScore()).thenReturn(0.0);
        Mockito.when(cam1.getConfidenceScore()).thenReturn(0.0);
        // Even though they nominally return a pose, the score disqualifies selection.
        Mockito.when(cam0.getEstimatedPose()).thenReturn(Optional.of(POSE_A));
        Mockito.when(cam1.getEstimatedPose()).thenReturn(Optional.of(POSE_B));
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        multiCam.periodic();

        assertTrue(multiCam.getEstimatedPose().isEmpty());
    }

    /**
     * Camera ordering in the constructor should not affect which pose is chosen —
     * the lower-index camera should win if it holds the higher confidence score.
     */
    @Test
    void getEstimatedPose_lowerIndexCamHasHigherScore_returnsLowerIndexCamPose() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        // cam0 (index 0) has the higher score even though it comes first in the list.
        Mockito.when(cam0.getConfidenceScore()).thenReturn(8.0);
        Mockito.when(cam0.getEstimatedPose()).thenReturn(Optional.of(POSE_A));
        Mockito.when(cam1.getConfidenceScore()).thenReturn(2.0);
        Mockito.when(cam1.getEstimatedPose()).thenReturn(Optional.of(POSE_B));
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        multiCam.periodic();

        assertEquals(Optional.of(POSE_A), multiCam.getEstimatedPose());
    }

    // ------------------------------------------------------------------
    // 4. getConfidenceScore()
    // ------------------------------------------------------------------

    /**
     * When no camera has a positive confidence score, {@code getConfidenceScore()}
     * returns {@code 0.0}.
     */
    @Test
    void getConfidenceScore_noCamsHaveLock_returnsZero() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        // Both mocks default to score 0.0.
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        multiCam.periodic();

        assertEquals(0.0, multiCam.getConfidenceScore());
    }

    /**
     * {@code getConfidenceScore()} returns the highest confidence score among all
     * cameras that have a lock (score > 0).
     */
    @Test
    void getConfidenceScore_returnsBestScoreAcrossCameras() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        Mockito.when(cam0.getConfidenceScore()).thenReturn(3.0);
        Mockito.when(cam1.getConfidenceScore()).thenReturn(7.0);
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        multiCam.periodic();

        assertEquals(7.0, multiCam.getConfidenceScore());
    }

    // ------------------------------------------------------------------
    // 5. hasTargetLock()
    // ------------------------------------------------------------------

    /**
     * If no camera reports {@code hasTargetLock() == true}, the aggregated result
     * must be {@code false}.
     */
    @Test
    void hasTargetLock_noCamHasLock_returnsFalse() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        // Both mocks default to hasTargetLock() == false.
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        multiCam.periodic();

        assertFalse(multiCam.hasTargetLock());
    }

    /**
     * If at least one camera reports {@code hasTargetLock() == true}, the
     * aggregated result must be {@code true} regardless of the other cameras.
     */
    @Test
    void hasTargetLock_atLeastOneCamHasLock_returnsTrue() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        // cam0 has no lock; cam1 does — OR should yield true.
        Mockito.when(cam1.hasTargetLock()).thenReturn(true);
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        multiCam.periodic();

        assertTrue(multiCam.hasTargetLock());
    }

    // ------------------------------------------------------------------
    // 6. hasMultiTagLock()
    // ------------------------------------------------------------------

    /**
     * If no camera reports {@code hasMultiTagLock() == true}, the aggregated result
     * must be {@code false}.
     */
    @Test
    void hasMultiTagLock_noCamHasMultiTag_returnsFalse() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        // Both mocks default to hasMultiTagLock() == false.
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        multiCam.periodic();

        assertFalse(multiCam.hasMultiTagLock());
    }

    /**
     * If one camera reports {@code hasMultiTagLock() == true} while another does
     * not, the aggregated result must be {@code true}.
     */
    @Test
    void hasMultiTagLock_oneCamHasMultiTag_returnsTrue() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        // cam0 has no multi-tag lock; cam1 does.
        Mockito.when(cam1.hasMultiTagLock()).thenReturn(true);
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        multiCam.periodic();

        assertTrue(multiCam.hasMultiTagLock());
    }

    // ------------------------------------------------------------------
    // 7. getVisibleTagIds()
    // ------------------------------------------------------------------

    /**
     * Tag IDs from all cameras are merged into a single list that is free of
     * duplicates and sorted in ascending order.
     */
    @Test
    void getVisibleTagIds_mergesAndSortsFromAllCameras() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        // cam0 sees tags 5 and 2; cam1 sees tags 7 and 1 — no overlap.
        Mockito.when(cam0.getVisibleTagIds()).thenReturn(List.of(5, 2));
        Mockito.when(cam1.getVisibleTagIds()).thenReturn(List.of(7, 1));
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        // getVisibleTagIds() does not need periodic(); it reads live from cams.
        List<Integer> result = multiCam.getVisibleTagIds();

        // Expect merged and sorted: [1, 2, 5, 7]
        assertEquals(List.of(1, 2, 5, 7), result);
    }

    /**
     * A tag ID seen by more than one camera should appear only once in the result.
     */
    @Test
    void getVisibleTagIds_deduplicatesSharedTagIds() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        // Both cameras see tag 3; cam1 also sees tag 6.
        Mockito.when(cam0.getVisibleTagIds()).thenReturn(List.of(3));
        Mockito.when(cam1.getVisibleTagIds()).thenReturn(List.of(3, 6));
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        List<Integer> result = multiCam.getVisibleTagIds();

        // Tag 3 must appear only once.
        assertEquals(List.of(3, 6), result);
    }

    /**
     * When no camera sees any tag, the returned list is empty (not null).
     */
    @Test
    void getVisibleTagIds_noTagsVisible_returnsEmptyList() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        // Both mocks default to returning an empty list.
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        List<Integer> result = multiCam.getVisibleTagIds();

        assertNotNull(result);
        assertTrue(result.isEmpty());
    }

    // ------------------------------------------------------------------
    // 8. getPrimaryTagTx() and getPrimaryTagId() — always camera 0
    // ------------------------------------------------------------------

    /**
     * {@code getPrimaryTagTx()} must always return the value from camera 0
     * (the forward-facing cam), even when camera 1 has a higher confidence score.
     */
    @Test
    void getPrimaryTagTx_alwaysDelegatesToCamera0() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        // cam1 has the better lock, but tx must still come from cam0.
        Mockito.when(cam0.getPrimaryTagTx()).thenReturn(-4.5);
        Mockito.when(cam1.getConfidenceScore()).thenReturn(9.0);
        Mockito.when(cam1.getPrimaryTagTx()).thenReturn(12.0);
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        multiCam.periodic();

        assertEquals(-4.5, multiCam.getPrimaryTagTx());
    }

    /**
     * {@code getPrimaryTagId()} must always return the tag ID from camera 0,
     * even when another camera has the stronger lock.
     */
    @Test
    void getPrimaryTagId_alwaysDelegatesToCamera0() {
        CamOdometryInterface cam0 = newMockCam();
        CamOdometryInterface cam1 = newMockCam();
        // cam1 has the better lock, but the primary tag ID must still come from cam0.
        Mockito.when(cam0.getPrimaryTagId()).thenReturn(4);
        Mockito.when(cam1.getConfidenceScore()).thenReturn(9.0);
        Mockito.when(cam1.getPrimaryTagId()).thenReturn(11);
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of(cam0, cam1));

        multiCam.periodic();

        assertEquals(4, multiCam.getPrimaryTagId());
    }

    // ------------------------------------------------------------------
    // 9. Empty-camera guard
    // ------------------------------------------------------------------

    /**
     * Calling {@code getPrimaryTagTx()} (or {@code getPrimaryTagId()}) on a
     * {@code MultiCamOdometry} constructed with an empty list must throw
     * {@link IllegalStateException}.
     */
    @Test
    void getPrimaryTagTx_noCamera_throwsIllegalStateException() {
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of());

        IllegalStateException ex = assertThrows(
            IllegalStateException.class, multiCam::getPrimaryTagTx);
        assertEquals("No cameras configured — at least one camera must be provided.", ex.getMessage());
    }

    /**
     * Calling {@code getPrimaryTagId()} on a {@code MultiCamOdometry}
     * constructed with an empty list must throw {@link IllegalStateException}.
     */
    @Test
    void getPrimaryTagId_noCamera_throwsIllegalStateException() {
        MultiCamOdometry multiCam = new MultiCamOdometry(List.of());

        IllegalStateException ex = assertThrows(
            IllegalStateException.class, multiCam::getPrimaryTagId);
        assertEquals("No cameras configured — at least one camera must be provided.", ex.getMessage());
    }
}
