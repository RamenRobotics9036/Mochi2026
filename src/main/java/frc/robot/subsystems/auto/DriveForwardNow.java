package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Manual autonomous command for Mochi2026.
 * Drives a set distance using Kraken motors and CTRE Swerve State.
 */
public class DriveForwardNow extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final Timer m_timer = new Timer();
  
  // CTRE specific request object for Kraken motors
  private final SwerveRequest.RobotCentric m_driveRequest = new SwerveRequest.RobotCentric();
  
  private double startX;
  private double startY;
  private final boolean resetOdometry;
  private final double distanceMeters;

  public DriveForwardNow(CommandSwerveDrivetrain drivetrain, double distanceMeters, boolean resetOdometry) {
    this.m_drivetrain = drivetrain;
    this.distanceMeters = distanceMeters;
    this.resetOdometry = resetOdometry;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    if (resetOdometry) {
      // Resets the Kraken odometry to face 90 degrees (Up-field)
      m_drivetrain.resetPose(new Pose2d(m_drivetrain.getState().Pose.getTranslation(), Rotation2d.fromDegrees(90)));
    }
    m_timer.restart();
    
    // Grab starting coordinates from the Phoenix 6 state
    startX = m_drivetrain.getState().Pose.getX();
    startY = m_drivetrain.getState().Pose.getY();
  }

  @Override
  public void execute() {
    // 1.0 m/s velocity. withVelocityX is Forward/Backward, withVelocityY is Left/Right
    m_drivetrain.setControl(m_driveRequest.withVelocityX(1.0).withVelocityY(0)); // Go Forward
  }

  @Override
  public boolean isFinished() {
    // Safety exit after 15 seconds
    if (m_timer.get() > 15.0) return true;

    double currentX = m_drivetrain.getState().Pose.getX();
    double currentY = m_drivetrain.getState().Pose.getY();
    
    // Calculate straight-line distance traveled
    double distanceTraveled = Math.sqrt(Math.pow(currentX - startX, 2) + Math.pow(currentY - startY, 2));
    
    return distanceTraveled >= distanceMeters;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the Kraken modules immediately
    m_drivetrain.setControl(new SwerveRequest.Idle());
  }
}
