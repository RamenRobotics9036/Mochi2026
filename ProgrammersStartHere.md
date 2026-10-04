# Programmers: Start Here
> This overview was written by Navneet Joneja, spelunking the codebase when he joined as a mentor after the 2026 season, with help from Claude.

Welcome to the Team 9036 (RamenRobotics) 2026 robot code - **Mochi**. This guide walks through
how the code is organized, how the pieces connect, and where to go when you need to change something.



The code is a **Java / WPILib command-based** project. It uses:

- **CTRE Phoenix 6** for the swerve drivetrain (TalonFX motors, CANcoders, Pigeon gyro)
- **REVLib** for the mechanism motors (SparkFlex / SparkMax)
- **Limelight** cameras for AprilTag vision (MegaTag2)
- **PathPlanner** for autonomous paths
- **PhotonVision** (photonlib), only to *simulate* the cameras on your laptop

---

## 1. Getting set up

Install the WPILib 2026 VS Code bundle, open this folder in WPILib VS Code, and let Gradle finish downloading.

These are the gradle build targets / commands, but an easier way to run most of them, assuming you have the WPILib VSCode, is just to run them using the WPIlib commands (Shft+Ctrl+P they type WPIlib and you'll see a list)

| Task | Command |
|---|---|
| Build (compiles and runs tests) | `./gradlew build` |
| Build without tests | `./gradlew assemble` |
| Run all unit tests | `./gradlew test` |
| Run one test class | `./gradlew test --tests "frc.robot.visutils.TestVisionKalmanFilter"` |
| Test coverage report | `./gradlew test jacocoTestReport -PjacocoEnabled` |
| Run the simulator on your laptop | `./gradlew simulateJava` |
| Deploy to the robot (connected to its network) | `./gradlew deploy` |

**Use the simulator.** Nearly everything (driving, vision, autos, mechanisms) runs in sim, so you can test
without the robot. See [Section 8](#8-simulation-sim).

---

## 2. The 30-second tour

```
src/main/java/frc/robot/
├── Main.java               Java entry point. Do not modify.
├── Robot.java              Robot lifecycle (init / auto / teleop / periodic loop)
├── RobotContainer.java     Creates everything and wires it together: subsystems, buttons, autos
├── Constants.java          Tunable numbers (speeds, CAN IDs, limits, ...)
├── JoystickInput.java      Turns driver sticks into drive speeds (deadband, slow mode, ...)
├── Telemetry.java          Publishes drivetrain data to dashboards
├── LimelightHelpers.java   This is generated once per project by the vendor. It can be added to in order 
|                           to add more vision capabilities.
| 
├── botconfig/              Which physical robot are we on? Per-robot settings.
├── subsystems/             The robot's mechanisms (drive, shooter, intake, ...)
│   ├── auto/               Autonomous chooser + PathPlanner glue
│   └── <mechanism>/        Real-hardware IO classes (XxxIoReal)
├── commands/               Actions the robot performs (shoot, intake, climb, align, ...)
├── visutils/               Vision: camera pose estimation, aiming, filters, dashboards (**currently broken, 
|                                   and not being used**)
├── sim/                    Physics simulation and simulated cameras
├── generated/              CTRE Tuner X swerve constants. Do not hand-edit.
└── util/                   Small helpers (MAC address lookup, math)

src/main/deploy/pathplanner/   PathPlanner paths (.path) and autos (.auto)
src/test/java/frc/robot/       JUnit tests
```

Other useful files in the repo root:

| Path | What it is |
|---|---|
| `plans/` | Design notes written before building features (vision Kalman filter, heartbeat, smooth drive, ...) |
| `robotinfo/` | Robot facts: gear ratios, Limelight settings, how robots are identified |
| `testpaths/` | Python tooling for checking paths |
| `coverage.py` | Helps read the JaCoCo coverage report |
| `elastic-layout-mochi.json` | Elastic dashboard layout used at competition |
| `simgui.json`, `simgui-ds.json` | Glass / sim GUI layouts |
| `tuner-project.json` | CTRE Tuner X project for the swerve |
| `vendordeps/` | Vendor library versions (Phoenix 6, REVLib, PathPlannerLib, photonlib) |

---

## 3. How the program runs

### `Robot.java`: the heartbeat

`Robot` extends WPILib's `TimedRobot`, which calls its methods on a fixed schedule:

| Method | When | What we do |
|---|---|---|
| `Robot()` constructor | Once at boot | Builds `RobotContainer` (which builds everything else) |
| `robotInit()` | Once at boot | Starts data logging; pre-loads PathPlanner autos (`AutoLogic.registerCommands()`) |
| `robotPeriodic()` | **Every 20 ms, in every mode** | Runs vision, then `CommandScheduler.run()`, then updates dashboards |
| `autonomousInit()` | Start of auto | Arms to **brake** mode; schedules the auto picked on the dashboard |
| `teleopInit()` | Start of teleop | Cancels auto; arms to **coast** mode |
| `testInit()` | Test mode | Cancels all commands |

Order inside `robotPeriodic()` (it matters):

1. `MotionlessTracker.update()` checks whether the robot is still. Movement resets the vision Kalman filter.
2. In sim only, `SimWrapper.robotPeriodic()` publishes fake Limelight data to NetworkTables first, so
   the rest of the loop sees fresh data.
3. Vision is enabled or disabled based on the dashboard toggle, then `MultiCamOdometry.periodic()` feeds
   camera poses into the drivetrain.
4. `CommandScheduler.getInstance().run()` runs every subsystem's `periodic()` and every active command.
5. Field display and dashboard telemetry update last, so they show this cycle's results.

### `RobotContainer.java`: where everything is wired up

If you want to know "where does X get created" or "what does button Y do", look here. The constructor:

1. Picks the robot config (`RobotIdentity.getBotConfig()`, see [Section 4](#4-multi-robot-support-botconfig)).
2. Builds every subsystem, choosing **real or simulated IO** for each (see [Section 5](#5-the-io-pattern-real-vs-sim-hardware)).
3. Creates the sim wrapper (null on the real robot), dashboards, and auto chooser.
4. Calls `configureDriveBindings()`, `configureOperateBindings()`, `configureDefaultCommands()`, and
   `registerNamedCommands()`.
5. Sets up the vision pipeline (`MotionlessTracker`, `MultiCamOdometry`).

---

## 4. Multi-robot support (`botconfig/`)

We have **two physical robots** that share this code:

| Robot | Config class | Swerve constants |
|---|---|---|
| Competition bot | `CompConfig` | `generated/GeneratedCompConstants.java` |
| Pancake (practice bot) | `PancakeConfig` | `generated/GeneratedPancakeConstants.java` |
| Simulation | `CompConfig` (default) | |

At startup, `RobotIdentity` reads the RoboRIO's **MAC address** and picks the matching config. An unknown
MAC logs an error and falls back to the competition config. To add or change a robot, see
`robotinfo/robotidentifiers.md` and the MAC constants in `RobotIdentity.java`.

Both configs implement `BotConfigInterface`, which provides:

- Swerve geometry and module constants, CAN bus, max speeds (`getSpeedAt12Volts`, `getSpeedInTeleop`)
- Camera list and robot-to-camera transforms (`getCameras()`)
- Vision options (`isVisionEnabledDefault`, `isMegaTag2Supported`, `isAutoVisionInjectionEnabled`)
- Alignment tolerances
- **`shouldForceDisableXxx()` switches** for shooter, indexer, spinny, climber, intake, and intake arm. If a
  mechanism is missing or broken on one robot, this makes the code use the *simulated* IO for it, so
  nothing crashes trying to talk to motors that aren't there.

> **Rule:** always go through `BotConfigInterface`. Never reference `CompConfig` or `PancakeConfig` directly
> in subsystem or command code.

---

## 5. The IO pattern (real vs. sim hardware)

Subsystems never talk to motors directly. They talk to an **IO interface**, and `RobotContainer` hands them
either a real or a simulated implementation:

```java
// RobotContainer.java
RollerIoInterface intakeIo =
    (Robot.isSimulation() || m_configInterface.shouldForceDisableIntake())
        ? SimIoFactory.createIntakeIoSim()
        : new IntakeIoReal();
IntakeSubsystem intakeSubsystem = new IntakeSubsystem(intakeIo);
```

The same subsystem logic then runs on the robot, in the simulator, and in unit tests.

Several mechanisms share a few **generic interfaces** (they live in `sim/`):

| Interface | Used by (real class) | Sim implementation |
|---|---|---|
| `RollerIoInterface` | Intake (`IntakeIoReal`), Indexer (`IndexerIoReal`), Spinny (`SpinnyIoReal`) | `sim/RollerSim/RollerIoSim` |
| `TwoMotorRollerIoInterface` | Shooter (`ShooterIoReal`) | `sim/RollerSim/TwoMotorRollerIoSim` |
| `ArmIoInterface` | Intake arm (`ArmIoReal`) | `sim/armsim/ArmIoSim` |
| `ElevatorIoInterface` | Climber (`ClimberIoReal`) | `sim/elevatorSim/ElevatorIoSim` |

Each interface has a small `DeviceOutputs` class for sensor readings (velocity, current, position). A
subsystem's `periodic()` calls `io.updateOutputs(outputs)` and then reads from `outputs`. Sim
implementations are created by `SimIoFactory`, using the `SimXxxConstants` classes in `Constants.java`.

The **drivetrain** is the exception: CTRE's `SwerveDrivetrain` handles real vs. sim itself.

---

## 6. Subsystems (`subsystems/`)

| Subsystem | Hardware | Notes |
|---|---|---|
| `CommandSwerveDrivetrain` | Phoenix 6 swerve (extends generated `TunerSwerveDrivetrain`) | Driving, pose estimation, `addVisionMeasurement`, SysId, `AlignToTag` |
| `ShooterSubsystem` | Two REV SparkFlex flywheel motors | |
| `HoodSubsystem` | Actuonix linear actuator (servo) | Adjusts the angle of the hood (`setAngle`) which can be used to tune the shot trajectory |
| `IndexerSubsystem` | One roller motor | Feeds fuel into the shooter |
| `IntakeSubsystem` | One roller motor | Has stall detection by current |
| `ArmSubsystem` | Two-motor intake arm | Position control and homing; `setIdleMode(IdleMode)` sets brake or coast |
| `ClimberSubsystem` | One-motor elevator | |
| `SpinnyWheels` | One always-on oscillating wheel | |
| `TestSubsystems` | | Helpers for test / bring-up |

### Autonomous (`subsystems/auto/`)

- `AutoLogic` is a static utility. It builds the auto chooser (`initShuffleboard`), pre-loads autos
  (`registerCommands`), and returns the selected command (`getSelectedAutoCommand`).
- **Named commands** are steps PathPlanner autos can call by name. They are registered in
  `RobotContainer.registerNamedCommands()`:

  | Name in PathPlanner | Command |
  |---|---|
  | `shoot` | `ShootCommand` |
  | `Full Auto Climb` | `FullAutoClimbCommand` |
  | `get fuel` | `GetFuelCommand` |
  | `set intake bottom` | `SetIntakeBottomCommand` |
  | `set intake top` | `SetIntakeTopCommand` |

  The names must match **exactly** (spaces and capitals included) between the code and the PathPlanner app.

- Paths and autos live in `src/main/deploy/pathplanner/`. Edit them with the PathPlanner desktop app.
  Auto names generally start with the starting position: `L_` (left), `C_` (center), `R_` (right).

---

## 7. Commands (`commands/`)

| Command | What it does |
|---|---|
| `ShootCommand` | Spin up the shooter, then feed with the indexer (used in autos) |
| `ShooterDefaultCommand` | Default for shooter and indexer. Operator **Y** shoots, **Right Bumper** reverses the indexer to clear jams |
| `ShooterTestCommand` | Manual shooter tuning (swap it in as the default when testing) |
| `IntakeCommand` | Run the intake rollers |
| `IntakeArmCommand` | Default for the arm. Operator triggers move it up or down |
| `IntakeArmHomeCommand` | Home the arm. **Incomplete** |
| `SetIntakeTopCommand` / `SetIntakeBottomCommand` | Move the arm to a preset position |
| `GetFuelCommand` | Arm down and intake, to collect fuel |
| `FullAutoClimbCommand` | Automated climb sequence **Not currently working as of 10/4/26** |
| `AlignToTagCommand` | Turns to line up with an AprilTag |
| `RotateToTargetCommand` | Rotate in place to face a target pose |
| `JiggleCommand` | Wiggle the drivetrain to shake loose stuck game pieces |
| `SpinnyDefaultCommand` | Keeps the spinny wheels running |

Many commands use a static `create(...)` factory instead of `new`, for example `ShootCommand.create(shooter, indexer)`.

---

## 8. Controller bindings

Two Xbox controllers. Bindings are in `RobotContainer.configureDriveBindings()`,
`configureOperateBindings()`, and the default commands.

### Driver (port 0)

| Input | Action |
|---|---|
| Left stick | Drive (field-centric translation) |
| Right stick X | Rotate |
| Right Bumper (hold) | Fine positioning: half-speed inputs |
| POV Up (hold) | Auto-aim heading at the primary AprilTag while you keep driving (`AimController`) |
| POV Down (hold) | Rotate in place to face the primary AprilTag (`RotateToTargetCommand`) |
| A (hold) | Brake (X-lock the wheels) |
| B (hold) | Point wheels in the left-stick direction (testing) |
| X (hold) | Limelight auto-approach: auto forward distance and heading, manual strafe |
| Y (hold) | Jiggle |
| Left Trigger | Align to AprilTag |
| Start | Re-seed field-centric (sets "forward" to the robot's current heading) |
| Back/Start + X/Y | SysId characterization routines (tuning only) |
| POV Right / POV Left | **Sim only:** inject odometry drift / cycle robot reset position |

### Operator (port 1)

| Input | Action |
|---|---|
| Y (hold) | Shoot: spin up the shooter, then feed with the indexer after a short delay |
| Right Bumper (hold) | Reverse the indexer (unjam) |
| A | Toggle the intake rollers |
| Left Trigger / Right Trigger | Move the intake arm one way / the other |
| Right Stick click | Home the intake arm |
| POV Up / POV Down (hold) | Climber up / down |
| POV Left / POV Right | Hood to 120° / 0° |

> If you change a binding, update this table and tell the drive team.

---

## 9. Vision (`visutils/`)
> ### WARNING
> This is currently not working and should be rewritten for the future. A couple vision commands are implemented in 
> LimelightHelpers, and those are all that are being currently used TTBOMK.

Vision corrects the drivetrain's position estimate on the field using AprilTags seen by Limelight cameras.

```
Limelight(s) ──NetworkTables──► SingleCamOdometry (one per camera)
                                        │
                                        ▼
                               MultiCamOdometry   picks the best-locked camera each cycle
                                        │
                     VisionInjectFilter decides whether the measurement is good enough
                                        │
                                        ▼
                     drivetrain.addVisionMeasurement(pose, timestamp)
```

| Class | Role |
|---|---|
| `SingleCamOdometry` | Reads one Limelight's MegaTag2 pose from NetworkTables |
| `MultiCamOdometry` | Loops over all cameras and picks the best-locked one |
| `MultiCamOdometryFactory` / `MultiCamOdometryWrapper` / `NoOpCamOdometry` | Build the right implementation; `NoOp` is used when there are no cameras |
| `CamOdometryInterface` | Shared interface for the above |
| `VisionInjectFilter` | Rejects bad or unreliable measurements before they reach the drivetrain |
| `MotionlessTracker` | Detects when the robot is stationary and for how long |
| `VisionKalmanFilter` | Very precise pose while stationary (needs at least 2 tags; resets when the robot moves) |
| `AimController` | PID heading control for aim-while-driving (driver POV Up) |
| `TurnToAngleHelper` | Turns tag IDs into field poses and angles to rotate to |
| `AllianceCalc` | Red/blue alliance math (field flipping) |
| `DriveSmooth` | Smooths joystick drive output |
| `VisionHeartBeat` | Detects when a camera stops sending data |
| `DriveAccuracyTester` | Drops "tape" markers on the field display to measure drive accuracy |
| `BasicInfoDashboard`, `DashboardFactory`, `ShowIcon` | Dashboard widgets, including the **vision on/off toggle** |
| `PerCycleState` | Per-loop cached state |

Camera names and mounting positions come from `BotConfigInterface.getCameras()`. Vision can be turned on and
off **live from the dashboard**, which helps if a camera starts giving bad data mid-match.

---

## 10. Simulation (`sim/`)

Run `./gradlew simulateJava` and use the Glass/Sim GUI (layout in `simgui.json`) with a controller plugged in.

| Class | Role |
|---|---|
| `SimWrapper` | One entry point for all sim features. **It is `null` on the real robot**, so always null-check it (`SimWrapper.create()`) |
| `GroundTruthSim` | Tracks where the robot *really* is, which can differ from where the robot *thinks* it is |
| `SimIoFactory` | Creates sim IOs for each mechanism |
| `RollerSim/`, `armsim/`, `elevatorSim/` | Physics-backed sim IOs (see [Section 5](#5-the-io-pattern-real-vs-sim-hardware)) |
| `visionproducers/VisionSim` | Uses PhotonVision to simulate which AprilTags each camera would see |
| `visionproducers/PhotonToLimelightConverter` + `LimelightTablePublisher` | Republish the simulated detections **as Limelight NetworkTables data**, so the vision code can't tell it's in sim |
| `ShowVisionOnField` | Draws vision poses, Kalman estimate, and tape markers on the Field2d widget |

**Testing vision in sim:** press driver **D-pad Right** to inject odometry drift (the robot's estimate
jumps away from the ground truth), then watch vision pull it back. **D-pad Left** cycles through reset positions.

---

## 11. Constants

- **`Constants.java`** holds team-tunable values in nested classes: `IntakeConstants`, `ArmConstants`,
  `ShooterConstants`, `IndexerConstants`, `SpinnyWheelsConstants`, `ClimberConstants`, `DriveConstants`,
  `VisionConstants`, `VisionKalmanConstants`, `AutoConstants`, plus `SimXxxConstants` for the simulator.
- **Per-robot values** (swerve, cameras, speed limits) belong in `CompConfig` / `PancakeConfig` through
  `BotConfigInterface`, not in `Constants.java`.
- **`generated/`** files come from **CTRE Tuner X**. Regenerate them with Tuner X instead of editing by hand.

---

## 12. Tests (`src/test/`)

JUnit tests cover most of the vision and sim logic: `TestSingleCamOdometry`, `TestMultiCamOdometry`,
`TestVisionKalmanFilter`, `TestMotionlessTracker`, `TestVisionInjectFilter`, `TestVisionHeartBeat`,
`TestTurnToAngleHelper`, `TestDriveAccuracyTester`, `TestGroundTruthSim`, `TestPhotonToLimelightConverter`,
`TestJoystickInput`, and `TestSpinnyWheels`.

Because of the IO pattern, you can test a subsystem by passing in a sim (or fake) IO. No robot is needed.
**Run `./gradlew test` before you push.**

---

## 13. Common tasks

### Add a new mechanism (subsystem)

1. Pick an existing IO interface (`RollerIoInterface`, `ArmIoInterface`, ...) or make a new one in `sim/`.
2. Write `XxxIoReal` in `subsystems/<mechanism>/` to talk to the real motor controllers.
3. If needed, write a sim IO and add a `createXxxIoSim()` method to `SimIoFactory`.
4. Write `XxxSubsystem extends SubsystemBase`. It takes the IO in its constructor and calls `updateOutputs`
   in `periodic()`.
5. Add constants to `Constants.java` (and `SimXxxConstants` if simulated).
6. Add a `shouldForceDisableXxx()` method to `BotConfigInterface` and both configs if the mechanism might not
   be on every robot.
7. Create it in `RobotContainer` with the real-vs-sim ternary, and bind buttons or a default command.

### Add a command PathPlanner can use

1. Write the command in `commands/`.
2. Register it in `RobotContainer.registerNamedCommands()`.
3. In the PathPlanner app, add a Named Command step with **exactly** the same name.

### Add or change an auto

Edit paths and autos with the PathPlanner app. They're saved in `src/main/deploy/pathplanner/`. Test in sim
by selecting the auto in the dashboard chooser and enabling Autonomous.

### Tune a value

Find it in `Constants.java` (or the bot config if it's robot-specific), change it, and test in sim first
when you can.

---

## 14. Conventions and tips

- **`$TODO`** comments mark known incomplete values or future work. Search for them when looking for something to do.
- **`$VISIONSIM`** comments mark simulation-specific vision code paths.
- `m_` prefix = member field. Constants use a `k` prefix (`kShootSpeed`). This breaks with standard Java code conventions, and we might retire this convention next season. (In future seasons, we should be using the standard Java conventions. The [Google Style Guide]( https://google.github.io/styleguide/javaguide.html) is a more complete version of the baseline Java conventions).
- Arms were switched by me (on Parker's request) to use **brake** mode in auto and **coast** mode in teleop (set in `Robot.autonomousInit()` and `teleopInit()`).
- **Home the intake arm** (operator right-stick click) before using arm position commands.
- New feature ideas are usually written up in `plans/` before they're built. Read the plans for background on vision features.
- Some comments in `RobotContainer` don't match the code (for example, the seed-field-centric comment says
  "Left Bumper" but the binding is **Start**). **Trust the code**, and fix the comment when you notice one.

Questions? Ask a mentor or one of the senior programmers. Then improve this document so the next person doesn't have to ask.
