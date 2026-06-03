package frc.robot.vision;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    private final List<CameraWrapper> cameras;
    private final CommandSwerveDrivetrain drivetrain;

    // NT-tunable std-dev parameters
    private final LiveTunable A_XY_MT1 = new LiveTunable("/vision/A_XY_MT1", VisionConstants.A_XY_MT1);
    private final LiveTunable A_XY_MT2 = new LiveTunable("/vision/A_XY_MT2", VisionConstants.A_XY_MT2);
    private final LiveTunable P_XY = new LiveTunable("/vision/P_XY", VisionConstants.P_XY);
    private final LiveTunable DEFENSE_SCALE =
        new LiveTunable("/vision/defenseStdDevScale", VisionConstants.DEFENSE_STD_DEV_SCALE);

    // Field visualisation
    private final Field2d visionField = new Field2d();
    private final Field2d rawField = new Field2d();

    // Per-camera online status (exposed for dashboard / auto-aim)
    public boolean[] cameraOnline;

    // State
    private boolean underDefense = false;
    private Pose2d lastAcceptedPose = null;
    private double lastAcceptTimestamp = 0;
    private Matrix<N3, N1> lastStdDevs = null;

    // Diagnostics
    private long framesProcessed = 0;
    private long framesAccepted = 0;

    public VisionSubsystem(List<CameraWrapper> cameras, CommandSwerveDrivetrain drivetrain) {
        this.cameras = cameras;
        this.drivetrain = drivetrain;
        this.cameraOnline = new boolean[cameras.size()];

        SmartDashboard.putData("VisionField", visionField);
        SmartDashboard.putData("RawVisionField", rawField);
    }

    /** Public accessor so auto-aim code can read current target data from any camera. */
    public List<CameraWrapper> getCameras() { return cameras; }

    public boolean isUnderDefense() { return underDefense; }

    public Pose2d getLastAcceptedPose() { return lastAcceptedPose; }

    public double getLastAcceptTimestamp() { return lastAcceptTimestamp; }

    /** Returns the std devs used for the last accepted measurement. */
    public Matrix<N3, N1> getLastStdDevs() { return lastStdDevs; }

    public long getFramesProcessed() { return framesProcessed; }
    public long getFramesAccepted() { return framesAccepted; }

    @Override
    public void periodic() {
        SwerveDriveState driveState = drivetrain.getState();
        ChassisSpeeds speeds = driveState.Speeds;
        Pose2d drivePose = driveState.Pose;

        // Update defense detection once per cycle
        underDefense = isUnderDefense(speeds, drivePose);

        SmartDashboard.putBoolean("/vision/underDefense", underDefense);

        for (int i = 0; i < cameras.size(); i++) {
            CameraWrapper cam = cameras.get(i);
            cameraOnline[i] = isCameraOnline(cam);

            if (!cameraOnline[i]) {
                continue;
            }

            // Provide robot orientation for MegaTag2
            double headingDeg = drivePose.getRotation().getDegrees();
            LimelightHelpers.SetRobotOrientation(cam.getName(), headingDeg, 0, 0, 0, 0, 0);

            RawFiducial[] rawFiducials = cam.getRawFiducials();
            if (rawFiducials == null || rawFiducials.length == 0) {
                continue;
            }

            double maxAmbiguity = computeMaxAmbiguity(rawFiducials);

            // Process MegaTag1
            LimelightHelpers.PoseEstimate mt1 = cam.getPoseEstimateMT1();
            if (mt1 != null && mt1.tagCount > 0) {
                framesProcessed++;
                processEstimate(cam, mt1, rawFiducials, maxAmbiguity, driveState, false);
            }

            // Process MegaTag2
            LimelightHelpers.PoseEstimate mt2 = cam.getPoseEstimateMT2();
            if (mt2 != null && mt2.tagCount > 0) {
                framesProcessed++;
                processEstimate(cam, mt2, rawFiducials, maxAmbiguity, driveState, true);
            }
        }

        // Publish diagnostics periodically
        visionField.setRobotPose(drivePose);
        SmartDashboard.putNumber("/vision/framesProcessed", framesProcessed);
        SmartDashboard.putNumber("/vision/framesAccepted", framesAccepted);
        if (framesProcessed > 0) {
            SmartDashboard.putNumber("/vision/acceptRatePct",
                100.0 * (double) framesAccepted / (double) framesProcessed);
        }
    }

    private void processEstimate(CameraWrapper cam, LimelightHelpers.PoseEstimate estimate,
                                 RawFiducial[] rawFiducials, double maxAmbiguity,
                                 SwerveDriveState driveState, boolean isMegaTag2) {

        // --- Gate 1: Stale data (same timestamp as last processed) ---
        if (estimate.timestampSeconds == cam.getLastTimestampSeconds()) {
            return;
        }

        Pose2d visionPose = estimate.pose;

        // --- Gate 2: Max ambiguity ---
        if (maxAmbiguity >= VisionConstants.MAX_AMBIGUITY) {
            return;
        }

        // --- Gate 3: Distance limits ---
        double maxDist = isMegaTag2 ? VisionConstants.MAX_DISTANCE_MT2 : VisionConstants.MAX_DISTANCE_MT1;
        if (estimate.tagCount == 1 && estimate.avgTagDist > maxDist) {
            return;
        }

        // --- Gate 4: Off-field ---
        if (!isPoseOnField(visionPose)) {
            return;
        }

        // --- Gate 5: Velocity plausibility (vision-to-vision implied speed) ---
        Pose2d lastPose = isMegaTag2 ? cam.getLastPoseMT2() : cam.getLastPoseMT1();
        if (!isVelocityPlausible(visionPose, estimate.timestampSeconds,
                                 lastPose, cam.getLastTimestampSeconds())) {
            return;
        }

        // --- Compute standard deviations ---
        Matrix<N3, N1> stdDevs;
        if (isMegaTag2) {
            stdDevs = computeStdDevsMT2(estimate, maxAmbiguity, rawFiducials);
        } else {
            stdDevs = computeStdDevsMT1(estimate, maxAmbiguity, rawFiducials);
        }

        if (stdDevs.get(0, 0) >= Double.MAX_VALUE) {
            return;
        }

        // --- Gate 6: Inter-tag spread consistency ---
        double spread = computeSpread(rawFiducials, visionPose);
        if (spread > VisionConstants.SPREAD_REJECT) {
            return;
        }

        // --- Inflate std devs by spread ---
        if (spread > VisionConstants.SPREAD_INFLATE_START) {
            double inflation = Math.pow(spread / VisionConstants.SPREAD_INFLATE_START, 2.0);
            stdDevs = stdDevs.times(inflation);
        }

        // --- Defense inflation: trust vision more under defense ---
        if (underDefense) {
            stdDevs = stdDevs.times(DEFENSE_SCALE.get());
        }

        // --- Show on raw field ---
        rawField.setRobotPose(visionPose);

        // --- Feed to drivetrain Kalman filter ---
        drivetrain.addVisionMeasurement(visionPose,
            estimate.timestampSeconds, stdDevs);

        // --- Maybe reset odometry if it walked off-field ---
        maybeResetToVision(visionPose, maxAmbiguity, estimate.tagCount);

        // --- Update per-camera state ---
        if (isMegaTag2) {
            cam.setLastPoseMT2(visionPose);
        } else {
            cam.setLastPoseMT1(visionPose);
        }
        cam.setLastTimestampSeconds(estimate.timestampSeconds);

        // Update global best
        if (estimate.timestampSeconds >= lastAcceptTimestamp) {
            lastAcceptedPose = visionPose;
            lastAcceptTimestamp = estimate.timestampSeconds;
            lastStdDevs = stdDevs;
        }

        framesAccepted++;
    }

    /** Detect if the robot is being actively defended (spun or pushed off position). */
    private boolean isUnderDefense(ChassisSpeeds speeds, Pose2d drivePose) {
        double omega = Math.abs(speeds.omegaRadiansPerSecond);
        double wheelSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        // Spin detection: high rotation with low translation = being spun
        boolean likelySpun = omega > VisionConstants.DEFENSE_SPIN_OMEGA
            && wheelSpeed < VisionConstants.DEFENSE_SPIN_MAX_TRANSLATION;

        // Position divergence: odometry has walked far from last trusted vision
        boolean positionDiverged = false;
        if (lastAcceptedPose != null) {
            double dt = Timer.getFPGATimestamp() - lastAcceptTimestamp;
            double disagreement = lastAcceptedPose.getTranslation()
                .getDistance(drivePose.getTranslation());
            double expectedDrift = VisionConstants.DEFENSE_DRIFT_RATE * dt
                + VisionConstants.DEFENSE_DIVERGE_BASE;
            positionDiverged = disagreement > expectedDrift;
        }

        return likelySpun || positionDiverged;
    }

    /** Compute max ambiguity across all raw fiducials. */
    private double computeMaxAmbiguity(RawFiducial[] rawFiducials) {
        double maxAmb = 0;
        for (RawFiducial rf : rawFiducials) {
            if (rf.ambiguity > maxAmb) maxAmb = rf.ambiguity;
        }
        return maxAmb;
    }

    /** RMS spread of per-tag distances vs the reported pose. */
    private double computeSpread(RawFiducial[] rawFiducials, Pose2d visionPose) {
        if (rawFiducials == null || rawFiducials.length < 2) return 0.0;
        double sumSqErr = 0;
        int count = 0;
        for (RawFiducial rf : rawFiducials) {
            double distFromPose = visionPose.getTranslation()
                .getDistance(new Translation2d(rf.distToRobot, 0));
            double err = distFromPose - rf.distToRobot;
            sumSqErr += err * err;
            count++;
        }
        return count > 0 ? Math.sqrt(sumSqErr / count) : 0.0;
    }

    /** Harmonic sum of 1/d^2 for all fiducials (higher = more confident). */
    private double computeHarmonicSum(RawFiducial[] rawFiducials) {
        if (rawFiducials == null || rawFiducials.length == 0) return 0.0;
        double sum = 0.0;
        for (RawFiducial rf : rawFiducials) {
            double dist = rf.distToRobot > 0.01 ? rf.distToRobot : rf.distToCamera;
            if (dist > 0.01) sum += 1.0 / (dist * dist);
        }
        return sum;
    }

    /** MT1 standard deviations: xy from distance, theta = PI/60 for multi-tag else inf. */
    private Matrix<N3, N1> computeStdDevsMT1(LimelightHelpers.PoseEstimate est,
                                             double maxAmbiguity, RawFiducial[] rawFiducials) {
        double ambInflation = 1.0 / Math.pow(1.0 - maxAmbiguity, 2.0);
        double harmonicSum = computeHarmonicSum(rawFiducials);
        if (harmonicSum <= 0) harmonicSum = 1.0 / (est.avgTagDist * est.avgTagDist + 1e-6);

        double xy = A_XY_MT1.get() * Math.pow(est.avgTagDist, P_XY.get())
            / Math.sqrt(harmonicSum) * ambInflation;
        double theta = est.tagCount == 1 ? Double.MAX_VALUE
            : VisionConstants.STD_DEVS_MT1_THETA * ambInflation;
        return VecBuilder.fill(xy, xy, theta);
    }

    /** MT2 standard deviations: xy from distance, theta always MAX_VALUE (heading from gyro). */
    private Matrix<N3, N1> computeStdDevsMT2(LimelightHelpers.PoseEstimate est,
                                             double maxAmbiguity, RawFiducial[] rawFiducials) {
        double ambInflation = 1.0 / Math.pow(1.0 - maxAmbiguity, 2.0);
        double harmonicSum = computeHarmonicSum(rawFiducials);
        if (harmonicSum <= 0) harmonicSum = 1.0 / (est.avgTagDist * est.avgTagDist + 1e-6);

        double xy = A_XY_MT2.get() * Math.pow(est.avgTagDist, P_XY.get())
            / Math.sqrt(harmonicSum) * ambInflation;
        return VecBuilder.fill(xy, xy, Double.MAX_VALUE);
    }

    /** Check if a pose is within the field boundaries (with margin). */
    private boolean isPoseOnField(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        return x >= -VisionConstants.FIELD_MARGIN
            && x <= VisionConstants.FIELD_LENGTH + VisionConstants.FIELD_MARGIN
            && y >= -VisionConstants.FIELD_MARGIN
            && y <= VisionConstants.FIELD_WIDTH + VisionConstants.FIELD_MARGIN;
    }

    /** Check that the robot didn't teleport implausibly fast between vision frames. */
    private boolean isVelocityPlausible(Pose2d currentPose, double currentTime,
                                        Pose2d lastPose, double lastTime) {
        if (lastPose == null || lastTime <= 0) return true;
        double dt = currentTime - lastTime;
        if (dt <= 0) return false;
        double dist = currentPose.getTranslation().getDistance(lastPose.getTranslation());
        double impliedSpeed = dist / dt;
        double maxSpeed = underDefense
            ? VisionConstants.MAX_VISION_IMPLIED_SPEED_DEFENSE
            : VisionConstants.MAX_VISION_IMPLIED_SPEED;
        return impliedSpeed < maxSpeed;
    }

    /** If odometry has drifted off-field and vision is trustworthy, reset XY only. */
    private void maybeResetToVision(Pose2d visionPose, double maxAmbiguity, int tagCount) {
        Pose2d odomPose = drivetrain.getState().Pose;
        boolean odomOffField = !isPoseOnField(odomPose);
        boolean visionTrusted = maxAmbiguity < VisionConstants.RESET_MAX_AMBIGUITY
            && tagCount >= VisionConstants.RESET_MIN_TAGS
            && isPoseOnField(visionPose);
        if (odomOffField && visionTrusted) {
            drivetrain.resetPose(new Pose2d(visionPose.getTranslation(), odomPose.getRotation()));
        }
    }

    /** Check if a Limelight has published a heartbeat recently. */
    public boolean isCameraOnline(CameraWrapper cam) {
        var table = NetworkTableInstance.getDefault().getTable(cam.getName());
        if (table == null) return false;
        long lastChange = table.getEntry("hb").getLastChange();
        if (lastChange == 0) return false;
        double lastChangeSecs = lastChange / 1_000_000.0;
        return (Timer.getFPGATimestamp() - lastChangeSecs) < VisionConstants.STALENESS_THRESHOLD_S;
    }
}
