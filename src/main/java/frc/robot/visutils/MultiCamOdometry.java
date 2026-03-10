package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.visutils.evaluateposes.EvaluatePosesInterface;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.TreeSet;
import java.util.function.BooleanSupplier;


/** Reads from multiple limelight cameras. */
public class MultiCamOdometry implements CamOdometryInterface {
    private final boolean m_megaTag2Enabled;
    private final boolean m_autoVisionInjectionEnabled;

    private final List<CamOdometryInterface> m_cameras;
    private final PerCycleState m_perCycleState = new PerCycleState();
    private final EvaluatePosesInterface m_evaluatePoses;

    /** Constructor. */
    public MultiCamOdometry(
        List<CamOdometryInterface> cameras,
        boolean megaTag2Enabled,
        boolean autoVisionInjectionEnabled,
        EvaluatePosesInterface evaluatePoses) {

        m_cameras = new ArrayList<>(cameras);
        m_megaTag2Enabled = megaTag2Enabled;
        m_autoVisionInjectionEnabled = autoVisionInjectionEnabled;
        m_evaluatePoses = evaluatePoses;
    }

    /**
     * Sets the dependencies needed for vision processing.
     *
     * @param filter The VisionKalmanFilter instance to inject measurements into
     * @param isMotionlessSupplier Supplier that returns true when robot is motionless
     */
    @Override
    public void setVisionDependenciesOnCamera(
            VisionKalmanFilter filter,
            BooleanSupplier isMotionlessSupplier) {

        for (CamOdometryInterface cam : m_cameras) {
            cam.setVisionDependenciesOnCamera(
                filter,
                isMotionlessSupplier);
        }
    }

    /**
     * Periodic update; should be called from robot periodic.
     * NOTE: For each cycle, track the camera with thehighest confidence
     *   score that has a target lock.  We also track the FIRST camera
     *   (in order) that has a target lock.
     */
    @Override
    public void setRobotOrientation() {
        if (isMegaTag2Enabled()) {
            for (CamOdometryInterface cam : m_cameras) {
                cam.setRobotOrientation_NoFlush();
            }
            LimelightHelpers.Flush();
        }
    }

    @Override
    public void setRobotOrientation_NoFlush() {
        if (isMegaTag2Enabled()) {
            for (CamOdometryInterface cam : m_cameras) {
                cam.setRobotOrientation_NoFlush();
            }
        }
    }

    @Override
    public void periodic() {
        // For Megatag2 support, we need to push the robot's current heading to the
        // Limelight every cycle.
        if (isMegaTag2Enabled()) {
            // This is expensive since it flushes NetworkTables on each call,
            // but we need to do it every cycle to keep the Limelight updated with
            // the robot's heading.  So we dont call it if vision or megatag2 is disabled.
            setRobotOrientation();
        }

        m_perCycleState.reset();
        for (CamOdometryInterface cam : m_cameras) {
            cam.periodic();

            // $TODO2 - Pick camera with highest confidence score each cycle
            // Only consider cameras that actually have a target lock;
            // without this guard a camera with score 0 beats the -1 reset sentinel.
            double singleCamScore = cam.getConfidenceScore();
            if (singleCamScore > 0 && singleCamScore > m_perCycleState.bestLockedCamScore) {
                m_perCycleState.bestLockedCam = Optional.of(cam);
                m_perCycleState.bestLockedCamScore = singleCamScore;
            }

            if (cam.hasTargetLock()) {
                m_perCycleState.hasTargetLock = true;
            }

            if (cam.hasMultiTagLock()) {
                m_perCycleState.hasMultiTagLock = true;
            }
        }
    }

    @Override
    public void enableVision(boolean enabled) {
        throw new UnsupportedOperationException("Vision enabling/disabling is not supported at the MultiCamOdometry level; enable/disable the entire wrapper instead.");
    }

    @Override
    public Optional<Transform2d> getVisionErrorAtSnapTime() {
        if (m_perCycleState.bestLockedCam.isPresent()) {
            return m_perCycleState.bestLockedCam.get().getVisionErrorAtSnapTime();
        }
        return Optional.empty();
    }

    @Override
    public Optional<Pose2d> getEstimatedPose() {
        // NOTE: We return the pose from the camera with the strongest lock, if it exists.
        if (m_perCycleState.bestLockedCam.isPresent()) {
            return m_perCycleState.bestLockedCam.get().getEstimatedPose();
        }
        else {
            return Optional.empty();
        }
    }

    @Override
    public double getConfidenceScore() {
        // NOTE: We return the confidence score from the camera with the strongest lock, if it exists.
        if (m_perCycleState.bestLockedCam.isPresent()) {
            return m_perCycleState.bestLockedCam.get().getConfidenceScore();
        }
        else {
            return 0.0;
        }
    }

    @Override
    public List<Integer> getVisibleTagIds() {
        TreeSet<Integer> seen = new TreeSet<>();
        for (CamOdometryInterface cam : m_cameras) {
            seen.addAll(cam.getVisibleTagIds());
        }
        return new ArrayList<>(seen);
    }

    @Override
    public boolean hasTargetLock() {
        return m_perCycleState.hasTargetLock;
    }

    @Override
    public boolean hasMultiTagLock() {
        return m_perCycleState.hasMultiTagLock;
    }

    @Override
    public boolean isLatestMt2() {
        if (m_perCycleState.bestLockedCam.isPresent()) {
            return m_perCycleState.bestLockedCam.get().isLatestMt2();
        }
        return false;
    }

    @Override
    public double getPrimaryTagTx() {
        // NOTE: Always returns Camera 0 forward-facing camera result, since
        // we use this to align robot rotationally to a target.
        return getPrimaryCam().getPrimaryTagTx();
    }

    @Override
    public int getPrimaryTagId() {
        // NOTE: Always returns Camera 0 forward-facing camera result, since
        // we use this to align robot rotationally to a target.
        return getPrimaryCam().getPrimaryTagId();
    }

    private CamOdometryInterface getPrimaryCam() {
        // Return the first cam in the list (the forward-facing cam).
        // Precondition: at least one camera must be provided.
        if (m_cameras.isEmpty()) {
            throw new IllegalStateException("No cameras configured — at least one camera must be provided.");
        }
        return m_cameras.get(0);
    }

    private boolean isMegaTag2Enabled() {
        return m_megaTag2Enabled;
    }
}
