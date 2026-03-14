package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;


/** No-op implementation of {@link CamOdometryInterface} that returns safe default values. */
class NoOpCamOdometry implements CamOdometryInterface {

    @Override
    public Optional<Pose2d> getEstimatedPose() {
        return Optional.empty();
    }

    @Override
    public OptionalDouble getVisionErrorAtSnapTime() {
        return OptionalDouble.empty();
    }

    @Override
    public double getConfidenceScore() {
        return 0.0;
    }

    @Override
    public List<Integer> getVisibleTagIds() {
        return Collections.emptyList();
    }

    @Override
    public boolean hasTargetLock() {
        return false;
    }

    @Override
    public boolean hasMultiTagLock() {
        return false;
    }

    @Override
    public boolean isLatestMt2() {
        return false;
    }

    @Override
    public int getPrimaryTagId() {
        return -1;
    }

    @Override
    public double getPrimaryTagTx() {
        return 0.0;
    }

    @Override
    public void setRobotOrientation() {}

    @Override
    public void setRobotOrientation_NoFlush() {}

    @Override
    public void periodic() {}

    @Override
    public void enableVision(boolean enabled) {}
}
