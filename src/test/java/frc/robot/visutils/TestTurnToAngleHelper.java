package frc.robot.visutils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Optional;
import java.util.OptionalDouble;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;


class TestTurnToAngleHelper {

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    // ── getTag2dPose ─────────────────────────────────────────────────

    /** Tag ID -1 is the sentinel for "no tag seen"; should return empty. */
    @Test
    void getTag2dPose_negativeOneReturnsEmpty() {
        assertTrue(TurnToAngleHelper.getTag2dPose(-1).isEmpty());
    }

    /** A valid tag ID that exists in the field layout should resolve to
     * a non-empty Translation2d. */
    @Test
    void getTag2dPose_validTagReturnsPosition() {
        // Tag 1 exists in every official FRC field layout
        Optional<Translation2d> result = TurnToAngleHelper.getTag2dPose(1);
        assertTrue(result.isPresent(), "Tag 1 should resolve");

        // Sanity-check the resolved 2D position has plausible field coordinates
        assertTrue(result.get().getX() > 0, "Tag X should be positive");
        assertTrue(result.get().getY() > 0, "Tag Y should be positive");
    }

    /** A tag ID that doesn't exist in the field layout should return empty. */
    @Test
    void getTag2dPose_unknownTagReturnsEmpty() {
        assertTrue(TurnToAngleHelper.getTag2dPose(9999).isEmpty());
    }

    // ── bearingToTag ─────────────────────────────────────────────────

    /** Bearing to a valid tag should point from the robot toward the tag's field position. */
    @Test
    void bearingToTag_validTag_returnsCorrectAngle() {
        // Place the robot at the origin; bearing should match atan2(tagY, tagX)
        var tagPos = TurnToAngleHelper.getTag2dPose(1).orElseThrow();
        Pose2d robotAtOrigin = new Pose2d();

        Rotation2d bearing = TurnToAngleHelper.bearingToTag(1, robotAtOrigin);
        double expected = Math.atan2(tagPos.getY(), tagPos.getX());
        assertEquals(expected, bearing.getRadians(), 1e-6);
    }

    /** An unknown tag ID should return Rotation2d.kZero as the fallback bearing. */
    @Test
    void bearingToTag_unknownTag_returnsZero() {
        Rotation2d bearing = TurnToAngleHelper.bearingToTag(9999, new Pose2d());
        assertEquals(0.0, bearing.getRadians(), 1e-6);
    }

    // ── bearingToPoint ───────────────────────────────────────────────

    /** A target directly ahead (+X) of the robot should produce a 0° bearing. */
    @Test
    void bearingToPoint_targetDirectlyAhead_returnsZero() {
        // Robot at origin, target at (3, 0) → bearing = atan2(0, 3) = 0°
        Rotation2d bearing = TurnToAngleHelper.bearingToPoint(
            3.0, 0.0, new Pose2d());
        assertEquals(0.0, bearing.getDegrees(), 1e-6);
    }

    /** A target to the left (+Y) at the same X should produce a 90° bearing. */
    @Test
    void bearingToPoint_targetToLeft_returns90Degrees() {
        // Robot at origin, target at (0, 5) → bearing = atan2(5, 0) = 90°
        Rotation2d bearing = TurnToAngleHelper.bearingToPoint(
            0.0, 5.0, new Pose2d());
        assertEquals(90.0, bearing.getDegrees(), 1e-6);
    }

    /** A target behind the robot (-X) should produce a ±180° bearing. */
    @Test
    void bearingToPoint_targetBehind_returns180Degrees() {
        // Robot at origin, target at (-4, 0) → bearing = atan2(0, -4) = ±180°
        Rotation2d bearing = TurnToAngleHelper.bearingToPoint(
            -4.0, 0.0, new Pose2d());
        assertEquals(180.0, Math.abs(bearing.getDegrees()), 1e-6);
    }

    // ── toOperatorFrame ──────────────────────────────────────────────

    /** On blue alliance (operator forward = 0°), the operator frame equals the field frame. */
    @Test
    void toOperatorFrame_blueAlliance_noChange() {
        Rotation2d fieldAngle = Rotation2d.fromDegrees(45);
        Rotation2d opForward = Rotation2d.fromDegrees(0);

        Rotation2d result = TurnToAngleHelper.toOperatorFrame(fieldAngle, opForward);
        assertEquals(45.0, result.getDegrees(), 1e-6);
    }

    /** On red alliance (operator forward = 180°), the operator frame is rotated 180°
     * from field frame. */
    @Test
    void toOperatorFrame_redAlliance_subtacts180() {
        Rotation2d fieldAngle = Rotation2d.fromDegrees(45);
        Rotation2d opForward = Rotation2d.fromDegrees(180);

        Rotation2d result = TurnToAngleHelper.toOperatorFrame(fieldAngle, opForward);
        // 45 − 180 = −135°
        assertEquals(-135.0, result.getDegrees(), 1e-6);
    }

    // ── bearingToPointInOperatorFrame ─────────────────────────────────

    /** Combines bearingToPoint and toOperatorFrame; verify a known case matches
     * doing them separately. */
    @Test
    void bearingToPointInOperatorFrame_matchesManualComposition() {
        Pose2d robot = new Pose2d(1, 1, new Rotation2d());
        Rotation2d opForward = Rotation2d.fromDegrees(180);

        Rotation2d manual = TurnToAngleHelper.toOperatorFrame(
            TurnToAngleHelper.bearingToPoint(4.0, 5.0, robot), opForward);
        Rotation2d combined = TurnToAngleHelper.bearingToPointInOperatorFrame(
            4.0, 5.0, robot, opForward);

        assertEquals(manual.getDegrees(), combined.getDegrees(), 1e-6);
    }

    // ── createAimPID ─────────────────────────────────────────────────

    /** The returned PID should have continuous input enabled across the ±π wrap. */
    @Test
    void createAimPid_hasContinuousInput() {
        PIDController pid = TurnToAngleHelper.createAimPID();
        // Continuous input means the PID wraps errors across ±π.
        // Verify by checking that a setpoint jump from −170° to +170°
        // produces a small negative output (shortest path is −20°), not
        // a large positive one (+340°).
        double output = pid.calculate(
            Math.toRadians(-170), Math.toRadians(170));
        assertTrue(output < 0,
            "PID should take the short path (negative) across the wrap");
    }

    /** The PID should have non-zero P and D gains. */
    @Test
    void createAimPid_hasNonZeroGains() {
        PIDController pid = TurnToAngleHelper.createAimPID();
        assertTrue(pid.getP() > 0, "P gain should be positive");
        assertTrue(pid.getD() > 0, "D gain should be positive");
    }

    // ── aimFeedforward ───────────────────────────────────────────────

    /** A stationary robot should produce zero feedforward regardless oftarget
     * position. */
    @Test
    void aimFeedforward_stationaryRobot_returnsZero() {
        double ff = TurnToAngleHelper.aimFeedforward(
            5.0, 3.0,
            new Pose2d(),
            new ChassisSpeeds(0, 0, 0));
        assertEquals(0.0, ff, 1e-9);
    }

    /** Driving perpendicular to the target vector should produce a non-zero feedforward. */
    @Test
    void aimFeedforward_perpendicularMotion_returnsNonZero() {
        // Robot at origin, target at (5, 0). Driving in +Y (perpendicular).
        // ff = (dx·vy − dy·vx) / (dx²+dy²) = (5·2 − 0) / 25 = 0.4
        double ff = TurnToAngleHelper.aimFeedforward(
            5.0, 0.0,
            new Pose2d(),
            new ChassisSpeeds(0, 2.0, 0));
        assertEquals(0.4, ff, 1e-9);
    }

    /** Driving directly toward the target should produce zero (or near-zero) feedforward. */
    @Test
    void aimFeedforward_radialMotion_returnsNearZero() {
        // Robot at origin, target at (5, 0). Driving in +X (radial).
        // ff = (dx·vy − dy·vx) / (dx²+dy²) = (5·0 − 0·3) / 25 = 0
        double ff = TurnToAngleHelper.aimFeedforward(
            5.0, 0.0,
            new Pose2d(),
            new ChassisSpeeds(3.0, 0, 0));
        assertEquals(0.0, ff, 1e-9);
    }

    /** When the robot is sitting on top of the target (dist < 0.1 m),
     * feedforward should be zero to avoid divide-by-zero. */
    @Test
    void aimFeedforward_onTarget_returnsZero() {
        // Robot at (3, 4), target at (3, 4) — distance is 0.
        // Even with non-zero velocity, the guard should return 0.
        double ff = TurnToAngleHelper.aimFeedforward(
            3.0, 4.0,
            new Pose2d(3.0, 4.0, new Rotation2d()),
            new ChassisSpeeds(2.0, 3.0, 0));
        assertEquals(0.0, ff, 1e-9);
    }

    // ── computeAimRateForTag ─────────────────────────────────────────

    /** An invalid tag ID (-1) should return empty rather than computing a rate. */
    @Test
    void computeAimRateForTag_invalidTag_returnsEmpty() {
        PIDController pid = TurnToAngleHelper.createAimPID();
        OptionalDouble result = TurnToAngleHelper.computeAimRateForTag(
            -1, new Pose2d(), new ChassisSpeeds(), pid, 5.0);
        assertTrue(result.isEmpty());
    }

    /** A valid nearby tag should return a non-empty aim rate. */
    @Test
    void computeAimRateForTag_validNearbyTag_returnsRate() {
        // Place robot 2 m from tag 1 (well within the 5 m limit)
        var tagPos = TurnToAngleHelper.getTag2dPose(1).orElseThrow();
        Pose2d nearTag = new Pose2d(
            tagPos.getX() - 2.0, tagPos.getY(), new Rotation2d());

        PIDController pid = TurnToAngleHelper.createAimPID();
        OptionalDouble result = TurnToAngleHelper.computeAimRateForTag(
            1, nearTag, new ChassisSpeeds(), pid, 5.0);
        assertTrue(result.isPresent(), "Should return a rate for a nearby valid tag");
    }

    /** A tag beyond kMaxAimDistanceMeters should be rejected as stale. */
    @Test
    void computeAimRateForTag_tagTooFarAway_returnsEmpty() {
        // Place robot 50 m from tag 1 — well beyond the 5 m limit
        var tagPos = TurnToAngleHelper.getTag2dPose(1).orElseThrow();
        Pose2d farAway = new Pose2d(
            tagPos.getX() - 50.0, tagPos.getY(), new Rotation2d());

        PIDController pid = TurnToAngleHelper.createAimPID();
        OptionalDouble result = TurnToAngleHelper.computeAimRateForTag(
            1, farAway, new ChassisSpeeds(), pid, 5.0);
        assertTrue(result.isEmpty(), "Tag beyond max distance should be rejected");
    }

    /** The returned rate should never exceed ±maxOmega. */
    @Test
    void computeAimRateForTag_resultIsClampedToMaxOmega() {
        // Place robot near tag 1 but facing the wrong way (180° error)
        // so the PID produces a large output that must be clamped.
        var tagPos = TurnToAngleHelper.getTag2dPose(1).orElseThrow();
        Pose2d nearTag = new Pose2d(
            tagPos.getX() - 2.0, tagPos.getY(),
            Rotation2d.fromDegrees(180));

        double maxOmega = 1.0; // deliberately small clamp
        PIDController pid = TurnToAngleHelper.createAimPID();
        OptionalDouble result = TurnToAngleHelper.computeAimRateForTag(
            1, nearTag, new ChassisSpeeds(), pid, maxOmega);

        assertTrue(result.isPresent());
        assertTrue(Math.abs(result.getAsDouble()) <= maxOmega + 1e-9,
            "Rate should be clamped to maxOmega");
    }

    // ── computeAimRate ───────────────────────────────────────────────

    /** When the robot is already facing the target, the PID output should be near zero. */
    @Test
    void computeAimRate_alreadyFacingTarget_nearZero() {
        // Robot at origin facing 0°, target at (5, 0) — already aligned.
        Translation2d target = new Translation2d(5.0, 0.0);
        Pose2d robot = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        PIDController pid = TurnToAngleHelper.createAimPID();

        double rate = TurnToAngleHelper.computeAimRate(
            target, robot, new ChassisSpeeds(), pid, 5.0);
        assertEquals(0.0, rate, 0.05, "Already facing target → near-zero rate");
    }

    /** When the target is 90° to the left, the rate should be positive (CCW). */
    @Test
    void computeAimRate_target90Left_positiveRate() {
        // Robot at origin facing 0°, target at (0, 5) — 90° CCW.
        Translation2d target = new Translation2d(0.0, 5.0);
        Pose2d robot = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        PIDController pid = TurnToAngleHelper.createAimPID();

        double rate = TurnToAngleHelper.computeAimRate(
            target, robot, new ChassisSpeeds(), pid, 10.0);
        assertTrue(rate > 0, "Target to the left → positive (CCW) rate");
    }

    /** The output should be clamped to ±maxOmega even if the heading error is very large. */
    @Test
    void computeAimRate_largeError_clampedToMaxOmega() {
        // Robot facing 0°, target behind at (-5, 0) — 180° error.
        Translation2d target = new Translation2d(-5.0, 0.0);
        Pose2d robot = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        double maxOmega = 0.5;
        PIDController pid = TurnToAngleHelper.createAimPID();

        double rate = TurnToAngleHelper.computeAimRate(
            target, robot, new ChassisSpeeds(), pid, maxOmega);
        assertTrue(Math.abs(rate) <= maxOmega + 1e-9,
            "Rate must be clamped to maxOmega");
    }

    /** With translational motion, the feedforward component should shift the output
     * relative to a stationary call. */
    @Test
    void computeAimRate_withTranslation_includesFeedforward() {
        // Robot at origin facing the target at (5, 0). Already aligned,
        // so the PID term is ~0. Driving in +Y adds a feedforward of
        // (5·2 − 0) / 25 = 0.4 rad/s.
        Translation2d target = new Translation2d(5.0, 0.0);
        Pose2d robot = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        PIDController pid = TurnToAngleHelper.createAimPID();

        double rateStationary = TurnToAngleHelper.computeAimRate(
            target, robot, new ChassisSpeeds(), pid, 5.0);

        // Reset PID so the D term doesn't carry over
        pid.reset();
        ChassisSpeeds moving = new ChassisSpeeds(0, 2.0, 0);
        double rateMoving = TurnToAngleHelper.computeAimRate(
            target, robot, moving, pid, 5.0);

        // The moving rate should differ from the stationary rate by ~0.4
        double ffDelta = rateMoving - rateStationary;
        assertEquals(0.4, ffDelta, 0.1,
            "Feedforward should shift the rate when driving perpendicular");
    }

    // ── AimController ────────────────────────────────────────────────

    /** When the aim button is not pressed, update() should return empty. */
    @Test
    void aimController_buttonNotPressed_returnsEmpty() {
        AimController aim = new AimController(5.0);
        OptionalDouble result = aim.update(
            false, 1, new Pose2d(), new ChassisSpeeds());
        assertTrue(result.isEmpty());
    }

    /** On the first cycle with the button pressed, the PID should reset and return a
     * rate for a valid tag. */
    @Test
    void aimController_risingEdge_resetsPidAndReturnsRate() {
        AimController aim = new AimController(5.0);
        // Place robot 2 m from tag 1
        var tagPos = TurnToAngleHelper.getTag2dPose(1).orElseThrow();
        Pose2d nearTag = new Pose2d(
            tagPos.getX() - 2.0, tagPos.getY(), new Rotation2d());

        OptionalDouble result = aim.update(
            true, 1, nearTag, new ChassisSpeeds());
        assertTrue(result.isPresent(),
            "First aim cycle with valid tag should return a rate");
    }

    /** When the target flickers away while the button is still held, update() returns empty but the
     * PID is NOT reset on the next valid cycle. */
    @Test
    void aimController_targetFlicker_pidNotReset() {
        AimController aim = new AimController(5.0);
        var tagPos = TurnToAngleHelper.getTag2dPose(1).orElseThrow();
        Pose2d nearTag = new Pose2d(
            tagPos.getX() - 2.0, tagPos.getY(),
            Rotation2d.fromDegrees(45)); // offset heading so PID accumulates state

        // Cycle 1: button pressed, valid tag → PID starts, returns rate
        OptionalDouble r1 = aim.update(true, 1, nearTag, new ChassisSpeeds());
        assertTrue(r1.isPresent());

        // Cycle 2: button still pressed, target lost (-1) → empty, but PID NOT reset
        OptionalDouble r2 = aim.update(true, -1, nearTag, new ChassisSpeeds());
        assertTrue(r2.isEmpty(), "Lost target should return empty");

        // Cycle 3: button still pressed, target returns → should get a rate
        // WITHOUT resetting the PID (the D-term history survives)
        OptionalDouble r3 = aim.update(true, 1, nearTag, new ChassisSpeeds());
        assertTrue(r3.isPresent(),
            "Target reappearing should produce a rate without PID reset");
    }

    /** Releasing and re-pressing the button should reset the PID again (fresh aiming session). */
    @Test
    void aimController_releaseAndRepress_resetsPidAgain() {
        AimController aim = new AimController(5.0);
        var tagPos = TurnToAngleHelper.getTag2dPose(1).orElseThrow();
        Pose2d nearTag = new Pose2d(
            tagPos.getX() - 2.0, tagPos.getY(),
            Rotation2d.fromDegrees(30));

        // Press: PID starts
        aim.update(true, 1, nearTag, new ChassisSpeeds());

        // Release: clears state
        OptionalDouble released = aim.update(
            false, 1, nearTag, new ChassisSpeeds());
        assertTrue(released.isEmpty(), "Released button should return empty");

        // Re-press: should get a fresh rate (PID was reset on rising edge)
        OptionalDouble repressed = aim.update(
            true, 1, nearTag, new ChassisSpeeds());
        assertTrue(repressed.isPresent(),
            "Re-press should produce a rate after PID reset");
    }
}
