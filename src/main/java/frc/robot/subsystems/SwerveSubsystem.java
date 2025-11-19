// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.CommandConstants.AlignRobotConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;
import frc.robot.ramenlib.logging.TriviaLogger;
import frc.robot.ramenlib.sim.simvision.VisionSim;
import frc.robot.ramenlib.sim.simvision.VisionSystemInterface;
import frc.robot.ramenlib.sim.simvision.VisionSystemSim;
import frc.robot.ramenlib.wheelcalibration.CalibrationOrchestrator;
import frc.robot.util.AutoLogic;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.photonvision.PhotonUtils;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class SwerveSubsystem extends SubsystemBase
{

  /**
   * Swerve drive object.
   */
  private final SwerveDrive         swerveDrive;
  /**
   * AprilTag field layout.
   */
  /**
   * Enable vision odometry updates while driving.
   */
  private final boolean trackOdometry = true;

  private Field2d m_field = new Field2d();

  private Pose2d m_targetPose = new Pose2d();
  
  // $TODO - We should see if we can achieve showing the target location on m_field, so
  // all the info is in one place.
  private Field2d m_targetField = new Field2d();

  private VisionSystemInterface m_vision = null;

  private boolean isUsingSimVision;

  private boolean m_isPathfinderWarmedUp = false;
  private boolean m_hasWarnedOnce = false;

  private CalibrationOrchestrator m_calibrationOrch = null;

  //
  // Simulated Vision
  //
  private VisionSim m_visionSim = null;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory)
  {
      // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
    //  In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
    //  The gear ratio is 6.75 motor revolutions per wheel rotation.[]
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.12);
    System.out.println("\"conversionFactors\": {");
    System.out.println("\t\"angle\": {\"factor\": " + angleConversionFactor + " },");
    System.out.println("\t\"drive\": {\"factor\": " + driveConversionFactor + " }");
    System.out.println("}");
 
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.kMaxSpeedMetersPerSecond,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                               Meter.of(4)),
                                                                             Rotation2d.fromDegrees(0)));
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
//    swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
    if (trackOdometry)
    {
      // Stop the odometry thread if we are using vision that way we can synchronize updates better.
      swerveDrive.stopOdometryThread();
    }
    setupPathPlanner();

    // IDO says he found this in YAGSL examples
    //

    swerveDrive.setMaximumAttainableSpeeds(swerveDrive.getMaximumChassisVelocity(), swerveDrive.getMaximumChassisAngularVelocity());
    swerveDrive.setMaximumAllowableSpeeds(swerveDrive.getMaximumChassisVelocity(), swerveDrive.getMaximumChassisAngularVelocity());
  }

  public void initShuffleboard() {
    if (!OperatorConstants.kCompetitionMode){
      ShuffleboardTab tabVision = Shuffleboard.getTab("Vision");
      tabVision.addDouble("TX", () -> m_vision.getTX());
      tabVision.addDouble("TY", () -> m_vision.getTY());
      tabVision.addBoolean("Is Detecting", () -> m_vision.isDetecting());
      tabVision.addDouble("ID", () -> m_vision.getID());

      tabVision.addBoolean("Is Using Sim Vision", () -> isUsingSimVision);

      tabVision.addDouble("Distance to target", () -> PhotonUtils.getDistanceToPose(
        m_vision.getRobotPose(),
        m_vision.getRelativeTargetPose().toPose2d()));
      
        tabVision.addDouble("Target Pose X", () -> m_targetPose.getX());
        tabVision.addDouble("Target Pose Y", () -> m_targetPose.getX());
        tabVision.addDouble("Target Pose Rot", () -> m_targetPose.getRotation().getDegrees());
    }

    if (!OperatorConstants.kCompetitionMode){
      ShuffleboardTab tab = Shuffleboard.getTab("Field");
      tab.add("Robot Position on Field", m_field);
      tab.add("Target Position on Field", m_targetField);
      tab.addBoolean("Path Planner Ready", () -> m_isPathfinderWarmedUp);

      // Add buttons to help callibrate wheel angles
      m_calibrationOrch = new CalibrationOrchestrator(swerveDrive);
      Command takeReadingCmd = new InstantCommand(() -> m_calibrationOrch.takeReading());
      tab.add("Take Wheel Reading", takeReadingCmd).withWidget("Command");

      Command showReportCmd = new InstantCommand(() -> m_calibrationOrch.showReport());
      tab.add("Show Report", showReportCmd).withWidget("Command");

      Command resetReadingsCmd = new InstantCommand(() -> m_calibrationOrch.resetReadings());
      tab.add("Reset Readings", resetReadingsCmd).withWidget("Command");
    }

    initLogging();
  }

  public void initLogging() {
    // Log current command on this subsystem.
    TriviaLogger logger = TriviaLogger.getInstance();
    logger.registerSubsystemCmdCallback(
      getSubsystem(),
      () -> (this.getCurrentCommand() == null) ? "None"
              : this.getCurrentCommand().getName());
  }

  @Override
  public void periodic()
  {
    // When vision is enabled we must manually update odometry in SwerveDrive
    if (trackOdometry)
    {
      swerveDrive.updateOdometry();
      if (m_vision.isDetecting()) {
        // $TODO - We originally thought that this call would update the swerve's location and
        // pose estimation whenever vision is detecting an apriltag.  We thought this would
        // help the swerve system adjust its own view of where the robot is on the field.
        // However, it doesnt seem to do that (otherwise when vision isdetecting, the robot
        // would slightly jump on the field).  So we should investigate this further.
        swerveDrive.addVisionMeasurement(m_vision.getRobotPose(), DriverStation.getMatchTime());
      }
      m_field.setRobotPose(getPose());
    }
  }

  public VisionSystemInterface getVisionSystem() {
    return m_vision;
  }

  public void updateVisionPose() {
    m_vision.updatePoses();
  }

  @Override
  public void simulationPeriodic()
  {
    m_visionSim.simulationPeriodic(getPose());

    var debugField = m_visionSim.getSimDebugField();
    debugField.getObject("EstimatedRobot").setPose(getPose());
    debugField.getObject("EstimatedRobotModules").setPoses(swerveDrive.getSwerveModulePoses(getPose()));
  }

  public Command alignWithAprilTagCommand(double transformDrive, double transformStrafe) {
    return runOnce(
      () -> {
        if (!m_vision.isDetecting()) {
          System.out.println("No vision target detected");
          return;
        }
    
        if (!m_isPathfinderWarmedUp) {
          System.out.println("Pathfinder not warmed up yet!");
          return;
        }
        
        Pose2d rawTargetPose = m_vision.getAbsoluteTargetPose().toPose2d();

        // NOTE: The last parameter to Twist2d must be in RADIANS.  This
        // fixed an important bug.
        Twist2d twistPose = new Twist2d(
          transformDrive,
          transformStrafe,
          rawTargetPose.getRotation().getRadians());

        Pose2d tempTargetPose = rawTargetPose.exp(twistPose);
        m_targetPose = new Pose2d(
          tempTargetPose.getX(),
          tempTargetPose.getY(),
          Rotation2d.fromDegrees(rawTargetPose.getRotation().getDegrees() + 180)
        );

        // Note: The idea here is that when we are aligning the robot based off of vision,
        // there's the possibility that the swerve pose on the field is innacurate, and
        // instead the vision system's estimation of robot location and pose on the field
        // is much better.  So we reset the robot position on the field to the vision
        // systems estimation.
        swerveDrive.resetOdometry(m_vision.getRobotPose());

        m_targetField.setRobotPose(m_targetPose);

        System.out.println("Driving to alignment with AprilTag");

        Command alignCmd = driveToPose(m_targetPose)
          .withTimeout(AlignRobotConstants.maxTimeSeconds)
          .withName("AlignAprilTag");
        
        alignCmd.schedule();
      }
    );
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {

            if (!m_isPathfinderWarmedUp && !m_hasWarnedOnce) {
              System.out.println("ERROR: Auto Pathplanner called, but not yet warmed up!");
              m_hasWarnedOnce = true;
            }

            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            return (OperatorConstants.kAlliance.get() == Alliance.Red);
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    
    // Run warmCommand, and then a command that sets m_isPathfinderWarmedUp to true.
    // NOTE: We cant use RunOnce() to set m_isPathfinderWarmedUp, since it
    // creates a dependency on the subsystem, and thus blocks the default comman d
    // from running (which means we cant drive the robot until warmup would be complete).
    m_isPathfinderWarmedUp = false;
    Command warmCommand =
      Commands.print("[--> START PATHFINDER WARMUP]")
      .andThen(PathfindingCommand.warmupCommand())
      .andThen(Commands.print("[--> *END* PATHFINDER WARMUP]"))
      .andThen(new InstantCommand(() -> m_isPathfinderWarmedUp = true))
      .ignoringDisable(true);

    warmCommand.schedule();
  }

  // /**
  //  * Get the distance to the speaker.
  //  *
  //  * @return Distance to speaker in meters.
  //  */
  // public double getDistanceToSpeaker()
  // {
  //   int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
  //   // Taken from PhotonUtils.getDistanceToPose
  //   Pose3d speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
  //   return getPose().getTranslation().getDistance(speakerAprilTagPose.toPose2d().getTranslation());
  // }

  // /**
  //  * Get the yaw to aim at the speaker.
  //  *
  //  * @return {@link Rotation2d} of which you need to achieve.
  //  */
  // public Rotation2d getSpeakerYaw()
  // {
  //   int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
  //   // Taken from PhotonUtils.getYawToPose()
  //   Pose3d        speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
  //   Translation2d relativeTrl         = speakerAprilTagPose.toPose2d().relativeTo(getPose()).getTranslation();
  //   return new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(swerveDrive.getOdometryHeading());
  // }

  // /**
  //  * Aim the robot at the speaker.
  //  *
  //  * @param tolerance Tolerance in degrees.
  //  * @return Command to turn the robot to the speaker.
  //  */
  // public Command aimAtSpeaker(double tolerance)
  // {
  //   SwerveController controller = swerveDrive.getSwerveController();
  //   return run(
  //       () -> {
  //         ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
  //                                                  controller.headingCalculate(getHeading().getRadians(),
  //                                                                              getSpeakerYaw().getRadians()),
  //                                                                      getHeading());
  //         drive(speeds);
  //       }).until(() -> Math.abs(getSpeakerYaw().minus(getHeading()).getDegrees()) < tolerance);
  // }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose)
  {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 1.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command align =  AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0)); // Goal end velocity in meters/sec

      align.addRequirements(this);
      return align;
  }

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to achieve.
   * @return {@link Command} to run.
   * @throws IOException    If the PathPlanner GUI settings is invalid
   * @throws ParseException If PathPlanner GUI settings is nonexistent.
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
  throws IOException, ParseException
  {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                                                                            swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint
        = new AtomicReference<>(new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                                                   swerveDrive.getStates(),
                                                   DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
                    () -> {
                      double newTime = Timer.getFPGATimestamp();
                      SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                                                                                      robotRelativeChassisSpeed.get(),
                                                                                      newTime - previousTime.get());
                      swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                                        newSetpoint.moduleStates(),
                                        newSetpoint.feedforwards().linearForces());
                      prevSetpoint.set(newSetpoint);
                      previousTime.set(newTime);

                    });
  }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds)
  {
    try
    {
      return driveWithSetpointGenerator(() -> {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());

      });
    } catch (Exception e)
    {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();

  }


  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand(boolean spin)
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, spin),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  @SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
  public Supplier<Command> driveMeters(double distanceInMeters, double speedInMetersPerSecond) {
    return () -> {
      double startX = getPose().getX();
      return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
      .until(() -> swerveDrive.getPose().getX() - startX >
                   distanceInMeters);
    };
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a given speed.
   *
   * @param distanceInMeters       the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per second
   * @return a Command that drives the swerve drive to a specific distance at a given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
  {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) >
                     distanceInMeters);
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
  {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX, boolean fieldRelative)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        fieldRelative,
                        false);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    }).withName("DriveFieldOriented");
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }


  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  public void trueResetPose() {
    swerveDrive.resetOdometry(m_vision.getRobotPose());
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance()
  {
    return OperatorConstants.kAlliance.get() == DriverStation.Alliance.Red;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance()
  {
    if (!isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        SwerveConstants.kMaxSpeedMetersPerSecond);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        SwerveConstants.kMaxSpeedMetersPerSecond);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  // public void addFakeVisionReading()
  // {
  //   swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  // }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}  
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

  public void stopSystem() {
  }
}