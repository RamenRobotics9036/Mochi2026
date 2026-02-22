package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.sim.visionproducers.VisionSimInterface;
import java.util.Optional;
import java.util.function.BooleanSupplier;


/** Reads from multiple limelight cameras. */
public class MultiCamOdometry {
    BotConfigInterface m_configInterface;
    private final LimelightOdometry m_singleCamlimelight;

    /** Constructor. */
    public MultiCamOdometry(
        BotConfigInterface configInterface,
        VisionSimInterface.EstimateConsumer poseConsumer) {

        m_configInterface = configInterface;

        m_singleCamlimelight = new LimelightOdometry(
            configInterface,
            poseConsumer);
    }

    /**
     * Sets the dependencies needed for vision processing.
     *
     * @param visionEnabledSupplier A BooleanSupplier returning true when vision is enabled
     * @param filter The VisionKalmanFilter instance to inject measurements into
     * @param isMotionlessSupplier Supplier that returns true when robot is motionless
     */
    public void setVisionDependencies(
            BooleanSupplier visionEnabledSupplier,
            VisionKalmanFilter filter,
            BooleanSupplier isMotionlessSupplier) {

        m_singleCamlimelight.setVisionDependencies(
            visionEnabledSupplier,
            filter,
            isMotionlessSupplier);
    }

    /** Periodic update; should be called from robot periodic. */
    public void periodic() {
        m_singleCamlimelight.periodic();
    }

    public Optional<Pose2d> getLatestVisPose() {
        return m_singleCamlimelight.getLatestVisPose();
    }

    public double getCurrentConfidenceScore() {
        return m_singleCamlimelight.getCurrentConfidenceScore();
    }

    public int getNumLockedTags() {
        return m_singleCamlimelight.getNumLockedTags();
    }

    public double getTx() {
        return m_singleCamlimelight.getTx();
    }

    public String getTargetList() {
        return m_singleCamlimelight.getTargetList();
    }

    /** Returns the ID of the last seen fiducial, or -1 if none. */
    public int getLastTarget() {
        return m_singleCamlimelight.getLastTarget();
    }
}
