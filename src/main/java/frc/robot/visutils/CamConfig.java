package frc.robot.visutils;

import frc.robot.visutils.evaluateposes.EvaluatePosesInterface;

/**
 * Static configuration for a {@link SingleCamOdometry} instance:
 * feature flags and the pose-evaluation strategy.
 */
public record CamConfig(
    boolean megaTag2Enabled,
    boolean autoVisionInjectionEnabled,
    EvaluatePosesInterface evaluatePoses) {}
