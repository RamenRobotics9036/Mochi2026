package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    private VisionConstants() { throw new UnsupportedOperationException("Utility class"); }

    // --- Camera configuration (comp bot defaults; tune in BotConfigInterface) ---

    // Default 2-Limelight config (front + back)
    public static final String CAM_FRONT = "limelight-front";
    public static final String CAM_BACK = "limelight-back";

    // Robot-to-camera transforms: x forward, y left, z up, roll/pitch/yaw in degrees
    public static final Transform3d FRONT_CAM_TRANSFORM = new Transform3d(
        new Translation3d(0.295, 0.0, 0.273),
        new Rotation3d(0, Units.degreesToRadians(-20), 0));
    public static final Transform3d BACK_CAM_TRANSFORM = new Transform3d(
        new Translation3d(-0.295, 0.0, 0.273),
        new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180)));

    // --- Std dev tuning (base values; tuned via NtTunableDouble on "/vision/") ---
    public static final double A_XY_MT1 = 0.09;
    public static final double A_XY_MT2 = 0.07;
    public static final double P_XY = 1.4;
    public static final double STD_DEVS_MT1_THETA = Math.PI / 60;

    // --- Gating limits ---
    public static final double MAX_AMBIGUITY = 0.4;
    public static final double MAX_DISTANCE_MT1 = 2.0;
    public static final double MAX_DISTANCE_MT2 = 5.0;
    public static final double STALENESS_THRESHOLD_S = 1.0;
    public static final double FIELD_LENGTH = 17.548;
    public static final double FIELD_WIDTH = 8.052;
    public static final double FIELD_MARGIN = 0.5;
    public static final double MAX_VISION_IMPLIED_SPEED = 7.0;
    public static final double MAX_VISION_IMPLIED_SPEED_DEFENSE = 12.0;

    // --- Spread check ---
    public static final double SPREAD_REJECT = 0.50;
    public static final double SPREAD_INFLATE_START = 0.10;

    // --- Odom reset ---
    public static final double RESET_MAX_AMBIGUITY = 0.15;
    public static final int RESET_MIN_TAGS = 2;

    // --- Auto-aim tuning ---
    // Rotation: P gain and deadband (in degrees TX)
    public static final double AIM_ROT_KP = 0.015;
    public static final double AIM_ROT_DEADBAND_DEG = 1.5;
    // Forward: P gain (TY error → fraction of MaxSpeed) and deadband (in degrees TY)
    public static final double AIM_FWD_KP = 0.04;
    public static final double AIM_FWD_DEADBAND_DEG = 1.0;
    public static final double AIM_TY_SETPOINT = 0.0;
    public static final double AIM_MAX_FWD_FRACTION = 0.4;

    // --- Defense detection ---
    public static final double DEFENSE_SPIN_OMEGA = 2.5;
    public static final double DEFENSE_SPIN_MAX_TRANSLATION = 1.0;
    public static final double DEFENSE_DIVERGE_BASE = 0.30;
    public static final double DEFENSE_DRIFT_RATE = 0.02;
    public static final double DEFENSE_STD_DEV_SCALE = 0.5;
}
