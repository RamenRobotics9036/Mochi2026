# Unit Test Coverage Gaps

Classes under `src/main/java/frc/robot/` that have non-trivial logic but no corresponding test file.

*Skipped: interfaces, IO stubs, generated code, constants-only classes, thin delegators/factories, `Robot.java`/`Main.java`, `RobotContainer.java`, `LimelightHelpers.java`*

---

## High Priority

| Class | Why it matters |
|---|---|
| `subsystems/ArmSubsystem.java` | Full `NOT_HOMED → HOMING → HOMED` state machine with stall detection, sustained-stall timer, overall timeout, soft limits, and pre-homing guards. Most complex untested logic in the project. |
| ~~`visutils/SingleCamOdometry.java`~~ | ~~`calculateEstimationStdDevs` (single- vs multi-tag scaling, >4 m hard rejection), `getConfidenceScore` exponential-decay mapping, ambiguity cutoff at 0.7, and duplicate-timestamp guard — these drive pose quality.~~ **Test file created (`TestSingleCamOdometry.java`); stubs pending implementation.** |
| `visutils/DriveSmooth.java` | `applyDeadbandWithRescale` linear rescaling, `applyResponseCurve` power curve, per-axis slew-rate limiting, `reset()` — pure math, no hardware. |

---

## Medium Priority

| Class | Why it matters |
|---|---|
| `visutils/BasicInfoDashboard.java` | `forceDisableVision`/`isVisionEnabled` interaction (force-disable always wins); debounce-with-memory on tag list. |
| `subsystems/auto/AutoLogic.java` | Null/manual routing in `getAutoCommand()`, `AutoTrajectoryProfile` duration accumulation across path segments. |
| `botconfig/RobotIdentity.java` | Detection fallback chain (comp MAC → pancake MAC → sim fallback → unknown default), lazy-init caching. |
| `util/MACAddress.java` | Byte-suffix alignment arithmetic — easy to get wrong, fully testable with synthetic byte arrays. |

---

## Lower Priority

| Class | Why it matters |
|---|---|
| `commands/JiggleCommand.java` | Timer-driven direction alternation logic. |
| `commands/SpinnyDefaultCommand.java` | Modular-clock CW/CCW switching; boundary at the threshold. |
| `visutils/ShowIcon.java` | Clear-all-then-set-one semantics, empty-list guard, index selection. |
| `subsystems/auto/DriveForwardNow.java` | Euclidean distance `isFinished()` + 15 s safety timeout. |
