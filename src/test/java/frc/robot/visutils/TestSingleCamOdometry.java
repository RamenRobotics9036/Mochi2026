package frc.robot.visutils;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.visutils.evaluateposes.EvaluatePosesMochiV1;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import robotutils.interfaces.SimLimelightProducerInterface;


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

        Consumer<SimLimelightProducerInterface.DrivetrainVisionPoseInfo> consumer =
            info -> {
                m_lastConsumedPose = info.pose();
                m_consumeCallCount++;
            };

        m_kalmanFilter = Mockito.mock(VisionKalmanFilter.class);

        // Identity transform — no rotation or translation from robot to camera.
        // Default test camera has MT2 enabled; motionless supplier defaults to false so
        // Kalman injection is not triggered unless the test explicitly requests it.
        m_cam = new SingleCamOdometry(
            CAM_NAME,
            new Transform3d(),
            new CamOutputs(
                consumer,
                m_kalmanFilter),
            new CamInputs(() -> makeState(POSE_A), null, () -> false),
            new CamConfig(true, true, new EvaluatePosesMochiV1()));
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
    private static final Pose2d POSE_B = new Pose2d(6.0, 1.0, Rotation2d.kPi);

    // ------------------------------------------------------------------
    // 0. MegaTag preference/fallback
    // ------------------------------------------------------------------

    /** Helper to create a camera with an explicit MT2 flag (defaults to POSE_A as current pose). */
    private SingleCamOdometry createCamWithMt2(boolean supportMegatag2) {
        return createCamWithMt2(supportMegatag2, () -> POSE_A);
    }

    private SingleCamOdometry createCamWithMt2(
        boolean supportMegatag2,
        Supplier<Pose2d> currentRobotPoseSupplier) {

        Consumer<SimLimelightProducerInterface.DrivetrainVisionPoseInfo> consumer =
            info -> {
                m_lastConsumedPose = info.pose();
                m_consumeCallCount++;
            };
        return new SingleCamOdometry(
            CAM_NAME,
            new Transform3d(),
            new CamOutputs(
                consumer,
                info -> {}),
            new CamInputs(() -> makeState(currentRobotPoseSupplier.get()), null, () -> false),
            new CamConfig(supportMegatag2, true, new EvaluatePosesMochiV1()));
    }

    private static SwerveDriveState makeState(Pose2d pose) {
        SwerveDriveState state = new SwerveDriveState();
        state.Pose = pose;
        return state;
    }

    /** Creates a camera wired to {@link #m_kalmanFilter} with the given motionless supplier. */
    private SingleCamOdometry createCamWithKalman(java.util.function.BooleanSupplier isMotionless) {
        Consumer<SimLimelightProducerInterface.DrivetrainVisionPoseInfo> consumer =
            info -> {
                m_lastConsumedPose = info.pose();
                m_consumeCallCount++;
            };
        return new SingleCamOdometry(
            CAM_NAME,
            new Transform3d(),
            new CamOutputs(
                consumer,
                m_kalmanFilter),
            new CamInputs(() -> makeState(POSE_A), null, isMotionless),
            new CamConfig(true, true, new EvaluatePosesMochiV1()));
    }

    @Test
    void periodic_megatag2Enabled_mt2MoreTags_prefersMt2() {
        SingleCamOdometry cam = createCamWithMt2(true, () -> POSE_B);
        PoseEstimate mt1 = singleTagEstimate(POSE_A, 2.0, 1.0, 0.0);
        PoseEstimate mt2 = new PoseEstimate(
            POSE_B, 1.0, 0.0, 2, 0.0, 2.0, 0.0,
            new RawFiducial[]{
                new RawFiducial(1, 0, 0, 0, 2.0, 2.0, 0.1),
                new RawFiducial(2, 0, 0, 0, 2.0, 2.0, 0.1)
            },
            true);

        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(mt1);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CAM_NAME))
            .thenReturn(mt2);

        cam.periodic();

        assertEquals(POSE_B, cam.getEstimatedPose().orElseThrow());
        assertEquals(POSE_B, m_lastConsumedPose);
        assertTrue(cam.hasMultiTagLock());
        assertTrue(cam.isLatestMt2());
    }

    @Test
    void periodic_megatag2Enabled_mt1MoreTags_prefersMt1() {
        SingleCamOdometry cam = createCamWithMt2(true, () -> POSE_A);
        // MT1 sees 3 tags at 2m
        RawFiducial f1 = new RawFiducial(1, 0, 0, 0, 2.0, 2.0, 0.1);
        RawFiducial f2 = new RawFiducial(2, 0, 0, 0, 2.0, 2.0, 0.1);
        RawFiducial f3 = new RawFiducial(3, 0, 0, 0, 2.0, 2.0, 0.1);
        PoseEstimate mt1 = new PoseEstimate(
            POSE_A, 1.0, 0.0, 3, 0.0, 2.0, 0.0,
            new RawFiducial[]{f1, f2, f3}, false);
        // MT2 sees 1 tag at 5m
        PoseEstimate mt2 = new PoseEstimate(
            POSE_B, 1.0, 0.0, 1, 0.0, 5.0, 0.0,
            new RawFiducial[]{ new RawFiducial(4, 0, 0, 0, 5.0, 5.0, 0.0) },
            true);

        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(mt1);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CAM_NAME))
            .thenReturn(mt2);

        cam.periodic();

        assertEquals(POSE_A, cam.getEstimatedPose().orElseThrow());
        assertEquals(POSE_A, m_lastConsumedPose);
        assertTrue(cam.hasMultiTagLock());
        assertFalse(cam.isLatestMt2());
    }

    @Test
    void periodic_megatag2Enabled_sameTagCount_mt1MuchCloser_prefersMt1() {
        SingleCamOdometry cam = createCamWithMt2(true, () -> POSE_A);
        // Both see 1 tag; MT1 at 2m, MT2 at 3.5m — gap exceeds the MT2 distance advantage
        PoseEstimate mt1 = singleTagEstimate(POSE_A, 2.0, 1.0, 0.0);
        PoseEstimate mt2 = new PoseEstimate(
            POSE_B, 1.0, 0.0, 1, 0.0, 3.5, 0.0,
            new RawFiducial[]{ new RawFiducial(7, 0, 0, 0, 3.5, 3.5, 0.0) },
            true);

        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(mt1);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CAM_NAME))
            .thenReturn(mt2);

        cam.periodic();

        assertEquals(POSE_A, cam.getEstimatedPose().orElseThrow());
        assertFalse(cam.isLatestMt2());
    }

    @Test
    void periodic_megatag2Enabled_sameTagCount_mt2SlightlyFarther_prefersMt2() {
        SingleCamOdometry cam = createCamWithMt2(true, () -> POSE_B);
        // Both see 1 tag; MT1 at 2.0m, MT2 at 2.3m — within the 0.5m MT2 advantage
        PoseEstimate mt1 = singleTagEstimate(POSE_A, 2.0, 1.0, 0.0);
        PoseEstimate mt2 = new PoseEstimate(
            POSE_B, 1.0, 0.0, 1, 0.0, 2.3, 0.0,
            new RawFiducial[]{ new RawFiducial(7, 0, 0, 0, 2.3, 2.3, 0.0) },
            true);

        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(mt1);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CAM_NAME))
            .thenReturn(mt2);

        cam.periodic();

        assertEquals(POSE_B, cam.getEstimatedPose().orElseThrow());
        assertTrue(cam.isLatestMt2());
    }

    @Test
    void periodic_megatag2Disabled_ignoresMegaTag2_usesMegaTag1() {
        SingleCamOdometry cam = createCamWithMt2(false);
        PoseEstimate mt1 = singleTagEstimate(POSE_A, 2.0, 1.0, 0.0);
        PoseEstimate mt2 = new PoseEstimate(
            POSE_B, 1.0, 0.0, 2, 0.0, 2.0, 0.0,
            new RawFiducial[]{
                new RawFiducial(1, 0, 0, 0, 2.0, 2.0, 0.1),
                new RawFiducial(2, 0, 0, 0, 2.0, 2.0, 0.1)
            },
            true);

        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(mt1);
        // MT2 is available but flag is off — should never be queried.
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CAM_NAME))
            .thenReturn(mt2);

        cam.periodic();

        assertEquals(POSE_A, cam.getEstimatedPose().orElseThrow());
        assertEquals(POSE_A, m_lastConsumedPose);
        assertTrue(cam.hasTargetLock());
        assertFalse(cam.isLatestMt2());
    }

    @Test
    void periodic_megatag2Enabled_noMt2Available_fallsBackToMegaTag1() {
        SingleCamOdometry cam = createCamWithMt2(true);
        PoseEstimate mt1 = singleTagEstimate(POSE_A, 2.0, 1.0, 0.0);

        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CAM_NAME))
            .thenReturn(null);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(mt1);

        cam.periodic();

        assertEquals(POSE_A, cam.getEstimatedPose().orElseThrow());
        assertEquals(POSE_A, m_lastConsumedPose);
        assertTrue(cam.hasTargetLock());
        assertFalse(cam.isLatestMt2());
    }

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
     * MT2 single-tag beyond 4 m must also be rejected — gyro fusion only resolves
     * rotational ambiguity, not translational accuracy at long range.
     */
    @Test
    void periodic_mt2SingleTag_beyondFourMeters_isRejected() {
        SingleCamOdometry cam = createCamWithMt2(true);
        PoseEstimate mt2 = new PoseEstimate(
            POSE_A, 1.0, 0.0, 1, 0.0, 5.0, 0.0,
            new RawFiducial[]{ new RawFiducial(7, 0, 0, 0, 5.0, 5.0, 0.0) },
            true);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(null);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CAM_NAME))
            .thenReturn(mt2);

        cam.periodic();

        assertFalse(cam.hasTargetLock());
        assertEquals(0.0, cam.getConfidenceScore());
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
        // Near estimate — small stdDevs → high confidence.
        PoseEstimate nearEst = multiTagEstimate(POSE_A, 1.0, 1.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(nearEst);
        m_cam.periodic();
        double nearConfidence = m_cam.getConfidenceScore();

        // Far estimate — stdDevs scaled up by (1 + dist² / 30) → lower confidence.
        PoseEstimate farEst = multiTagEstimate(POSE_A, 6.0, 2.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(farEst);
        m_cam.periodic();
        double farConfidence = m_cam.getConfidenceScore();

        assertTrue(farConfidence < nearConfidence,
            "Confidence at 6 m ("
            + farConfidence
            + ") should be lower than at 1 m ("
            + nearConfidence + ")");
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
        // ambiguity 0.71 > 0.7 → lock must be cleared.
        PoseEstimate est = singleTagEstimate(POSE_A, 2.0, 1.0, 0.71);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        m_cam.periodic();

        assertFalse(m_cam.hasTargetLock());
        assertFalse(m_cam.hasMultiTagLock());
    }

    /**
     * A single-tag estimate with {@code ambiguity == 0.7} (exactly at the threshold) must
     * be accepted because the rejection condition is strictly {@code ambiguity > 0.7}.
     */
    @Test
    void periodic_singleTag_ambiguityExactlyAtThreshold_isAccepted() {
        // ambiguity == 0.7; condition is strictly > 0.7, so this must pass through.
        PoseEstimate est = singleTagEstimate(POSE_A, 2.0, 1.0, 0.7);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        m_cam.periodic();

        assertTrue(m_cam.hasTargetLock());
        assertTrue(m_cam.getConfidenceScore() > 0.0);
    }

    /**
     * The ambiguity check only applies when there is exactly one raw fiducial.  A
     * multi-tag estimate must be accepted even if one of the fiducials reports high ambiguity.
     */
    @Test
    void periodic_multiTag_highAmbiguity_isStillAccepted() {
        // tagCount == 2 → the single-tag ambiguity guard is not triggered.
        RawFiducial f1 = new RawFiducial(3, 0, 0, 0, 2.0, 2.0, 0.9);
        RawFiducial f2 = new RawFiducial(5, 0, 0, 0, 2.0, 2.0, 0.9);
        PoseEstimate est = new PoseEstimate(POSE_A, 1.0, 0.0,
            2, 0.0, 2.0, 0.0, new RawFiducial[]{f1, f2}, false);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        m_cam.periodic();

        assertTrue(m_cam.hasTargetLock());
        assertTrue(m_cam.hasMultiTagLock());
        assertTrue(m_cam.getConfidenceScore() > 0.0);
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
        // Both estimates share timestamp 1.0 — second call must be skipped.
        PoseEstimate est = singleTagEstimate(POSE_A, 2.0, 1.0, 0.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        m_cam.periodic();
        m_cam.periodic();

        // Consumer must have fired exactly once.
        assertEquals(1, m_consumeCallCount);
    }

    /**
     * Different timestamps on successive cycles must each be processed independently —
     * the consumer must receive two calls.
     */
    @Test
    void periodic_differentTimestamps_bothCallsAreProcessed() {
        PoseEstimate est1 = singleTagEstimate(POSE_A, 2.0, 1.0, 0.0);
        PoseEstimate est2 = singleTagEstimate(POSE_A, 2.0, 2.0, 0.0);

        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est1);
        m_cam.periodic();

        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est2);
        m_cam.periodic();

        assertEquals(2, m_consumeCallCount);
    }

    // ------------------------------------------------------------------
    // 6. Vision enabled / disabled
    // ------------------------------------------------------------------

    /**
     * When the vision-enabled supplier returns {@code true} the consumer must be called
     * with the correct pose.
     */
    @Test
    void periodic_visionEnabled_consumerIsCalled() {
        // enableVision is not supported at the SingleCamOdometry level; verify it throws.
        UnsupportedOperationException ex = assertThrows(
            UnsupportedOperationException.class, () -> m_cam.enableVision(true));
        assertTrue(ex.getMessage().contains("SingleCamOdometry"),
            "Exception message should mention SingleCamOdometry, got: " + ex.getMessage());
    }

    // ------------------------------------------------------------------
    // 7. Kalman filter injection
    // ------------------------------------------------------------------

    /**
     * When the robot is motionless AND the estimate has ≥ 2 tags, the Kalman filter's
        * Kalman measurement consumer must be called exactly once with the correct pose and
        * tag count.
     */
    @Test
    void periodic_motionlessAndMultiTag_injectsKalman() {
        UnsupportedOperationException ex = assertThrows(
            UnsupportedOperationException.class, () -> m_cam.enableVision(true));
        assertTrue(ex.getMessage().contains("SingleCamOdometry"),
            "Exception message should mention SingleCamOdometry, got: " + ex.getMessage());

        SingleCamOdometry cam = createCamWithKalman(() -> true);

        PoseEstimate est = multiTagEstimate(POSE_A, 2.0, 1.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        cam.periodic();

        Mockito.verify(m_kalmanFilter, Mockito.times(1))
            .accept(Mockito.argThat(info -> info.pose().equals(POSE_A) && info.tagCount() == 2));
    }

    /**
     * When the robot is moving (motionless supplier returns {@code false}), the Kalman filter
     * must NOT be injected, even for a multi-tag estimate.
     */
    @Test
    void periodic_movingAndMultiTag_doesNotInjectKalman() {
        UnsupportedOperationException ex = assertThrows(
            UnsupportedOperationException.class, () -> m_cam.enableVision(true));
        assertTrue(ex.getMessage().contains("SingleCamOdometry"),
            "Exception message should mention SingleCamOdometry, got: " + ex.getMessage());

        SingleCamOdometry cam = createCamWithKalman(() -> false);

        PoseEstimate est = multiTagEstimate(POSE_A, 2.0, 1.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        cam.periodic();

        Mockito.verify(m_kalmanFilter, Mockito.never())
            .accept(Mockito.any());
    }

    /**
     * When the robot is motionless but the estimate has only a single tag, the Kalman filter
     * must NOT be injected.
     */
    @Test
    void periodic_motionlessAndSingleTag_doesNotInjectKalman() {
        UnsupportedOperationException ex = assertThrows(
            UnsupportedOperationException.class, () -> m_cam.enableVision(true));
        assertTrue(ex.getMessage().contains("SingleCamOdometry"),
            "Exception message should mention SingleCamOdometry, got: " + ex.getMessage());

        SingleCamOdometry cam = createCamWithKalman(() -> true);

        PoseEstimate est = singleTagEstimate(POSE_A, 2.0, 1.0, 0.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        cam.periodic();

        Mockito.verify(m_kalmanFilter, Mockito.never())
            .accept(Mockito.any());
    }

    /**
     * Verifies that calling {@code periodic()} with a valid multi-tag estimate does not throw,
     * even when the motionless supplier returns false (Kalman injection is skipped).
     */
    @Test
    void periodic_multiTag_doesNotThrow() {
        // No setVisionDependencies call — Kalman filter remains null.
        PoseEstimate est = multiTagEstimate(POSE_A, 2.0, 1.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        assertDoesNotThrow(() -> m_cam.periodic());
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
        // Single-tag beyond 4 m → stdDevs = MAX_VALUE → confidence must be exactly 0.
        PoseEstimate est = singleTagEstimate(POSE_A, 5.0, 1.0, 0.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        m_cam.periodic();

        assertEquals(0.0, m_cam.getConfidenceScore());
    }

    /**
     * A near-zero-distance single-tag estimate produces a very small combined positional
     * uncertainty.  The resulting confidence score must be close to 100 (≥ 90).
     */
    @Test
    void getConfidenceScore_nearZeroDistance_returnsHighScore() {
        // kMultiTagStdDevs = (0.5, 0.5, 1); at dist≈0 scale≈1 → posUncertainty≈0.707
        // confidence = 100 * exp(-0.707/2) ≈ 70 → well above 50.
        PoseEstimate est = multiTagEstimate(POSE_A, 0.01, 1.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        m_cam.periodic();

        assertTrue(m_cam.getConfidenceScore() >= 50.0,
            "Expected score >= 50 at near-zero distance, got: " + m_cam.getConfidenceScore());
    }

    /**
     * A multi-tag estimate at moderate distance produces a confidence score strictly between
     * 0 and the near-zero-distance score, confirming exponential decay with distance.
     */
    @Test
    void getConfidenceScore_multiTagModerateDistance_returnsIntermediateScore() {
        // Multi-tag at near-zero gives highest score (≈70).
        PoseEstimate nearEst = multiTagEstimate(POSE_A, 0.01, 1.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(nearEst);
        m_cam.periodic();
        double maxScore = m_cam.getConfidenceScore();

        // Multi-tag at 3 m: scale = 1 + 9/30 = 1.3 → higher uncertainty → lower score (≈63).
        PoseEstimate modEst = multiTagEstimate(POSE_A, 3.0, 2.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(modEst);
        m_cam.periodic();
        double midScore = m_cam.getConfidenceScore();

        assertTrue(midScore > 0.0 && midScore < maxScore,
            "Expected 0 < midScore (" + midScore + ") < maxScore (" + maxScore + ")");
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
        // multiTagEstimate uses fiducial IDs 3 and 5.
        PoseEstimate est = multiTagEstimate(POSE_A, 2.0, 1.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        m_cam.periodic();

        assertEquals(List.of(3, 5), m_cam.getVisibleTagIds());
    }

    /**
     * {@code getPrimaryTagId()} must return the ID of the first raw fiducial in the pose
     * estimate (index 0).
     */
    @Test
    void getPrimaryTagId_afterLock_returnsFirstFiducialId() {
        // multiTagEstimate: first fiducial has ID 3.
        PoseEstimate est = multiTagEstimate(POSE_A, 2.0, 1.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        m_cam.periodic();

        assertEquals(3, m_cam.getPrimaryTagId());
    }

    /**
     * When no targets are visible, {@code getPrimaryTagId()} should return {@code -1}
     * (the "none" sentinel) and {@code getVisibleTagIds()} should return an empty list.
     */
    @Test
    void getVisibleTagIds_noTargets_returnsEmpty_andPrimaryTagIdIsMinusOne() {
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(null);

        m_cam.periodic();

        assertTrue(m_cam.getVisibleTagIds().isEmpty());
        assertEquals(-1, m_cam.getPrimaryTagId());
    }

    /**
     * {@code getPrimaryTagTx()} must return the value from {@code LimelightHelpers.getTX()}
     * for the camera's name after a successful lock.
     */
    @Test
    void getPrimaryTagTx_afterLock_returnsLimelightTxValue() {
        m_limelightMock.when(() -> LimelightHelpers.getTX(CAM_NAME)).thenReturn(12.5);

        PoseEstimate est = singleTagEstimate(POSE_A, 2.0, 1.0, 0.0);
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(est);

        m_cam.periodic();

        assertEquals(12.5, m_cam.getPrimaryTagTx());
    }

    /**
     * {@code getPrimaryTagTx()} must return {@code 0.0} when no targets are visible (cleared
     * state).
     */
    @Test
    void getPrimaryTagTx_noTargets_returnsZero() {
        m_limelightMock.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(CAM_NAME))
            .thenReturn(null);

        m_cam.periodic();

        assertEquals(0.0, m_cam.getPrimaryTagTx());
    }
}
