package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.TreeSet;
import java.util.function.BooleanSupplier;


/** Reads from multiple limelight cameras. */
public class MultiCamOdometry implements CamOdometryInterface {
    private final List<CamOdometryInterface> m_cameras;
    private final PerCycleState m_perCycleState = new PerCycleState();

    /** Constructor. */
    public MultiCamOdometry(List<CamOdometryInterface> cameras) {
        m_cameras = new ArrayList<>(cameras);
    }

    /**
     * Sets the dependencies needed for vision processing.
     *
     * @param visionEnabledSupplier A BooleanSupplier returning true when vision is enabled
     * @param filter The VisionKalmanFilter instance to inject measurements into
     * @param isMotionlessSupplier Supplier that returns true when robot is motionless
     */
    @Override
    public void setVisionDependencies(
            BooleanSupplier visionEnabledSupplier,
            VisionKalmanFilter filter,
            BooleanSupplier isMotionlessSupplier) {

        for (CamOdometryInterface cam : m_cameras) {
            cam.setVisionDependencies(
                visionEnabledSupplier,
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
    public void periodic() {
        int num = 0;

        m_perCycleState.reset();

        for (CamOdometryInterface cam : m_cameras) {
            cam.periodic();

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
}
