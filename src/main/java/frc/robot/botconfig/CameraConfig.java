package frc.robot.botconfig;

import edu.wpi.first.math.geometry.Transform3d;

/** Describes a single Limelight camera on the robot. */
public record CameraConfig(String name, Transform3d robotToCamera) {}
