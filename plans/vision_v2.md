# Vision V2 for Mochi bot

Team 2025 has a good example of using Megatag2:
https://github.com/FRC2713/Robot2025/blob/e1183247b14d990c0cafb002323b2e4e0ea10770/src/main/java/frc/robot/subsystems/vision/VisionIOLimelights.java#L44


Discard vision data if robot is spinning, for example:
    if (RobotContainer.driveSubsystem.getAngularVelocityRadPerSec() > Units.degreesToRadians(720)) {
      this.state = CombinedMegaTagState.REJECTED_DUE_TO_SPIN_BLUR;
    }



Example to set camera position for limelight:
@Builder
public class LimelightInfo {
  public enum MountingDirection {
    VERTICAL_LL3(49.7, 63.3),
    HORIZONTAL_LL3(63.3, 49.7); // MegaTag2 only supports Horizontal

    @Getter private final double horizontalFOV;
    @Getter private final double verticalFOV;

    MountingDirection(double horizontalFOV, double verticalFOV) {
      this.horizontalFOV = horizontalFOV;
      this.verticalFOV = verticalFOV;
    }
  }

  @Getter private String ntTableName;
  @Getter private Transform3d location;
  @Getter private MountingDirection mountingDirection;

  public void setCameraPose_RobotSpace() {
    LimelightHelpers.setCameraPose_RobotSpace(
        this.ntTableName,
        this.location.getX(),
        this.location.getY(),
        this.location.getZ(),
        Units.radiansToDegrees(this.location.getRotation().getX()),
        Units.radiansToDegrees(this.location.getRotation().getY()),
        Units.radiansToDegrees(this.location.getRotation().getZ()));
  }
}
