## Mochi2026 — Copilot / AI contribution notes

These notes are intended to help an AI coding assistant be immediately productive in this WPILib Java robot project.

High level
- This repository is a WPILib/GradleRIO Java FRC robot project (see `build.gradle` — plugin `edu.wpi.first.GradleRIO`).
- Main application code lives under `src/main/java/frc/robot/` (packages: `commands`, `subsystems`, `auto`, `generated`).
- Robot constants are centralized in `src/main/java/frc/robot/Constants.java` and use nested static classes (e.g. `ElevatorConstants`, `VisionConstants`).

Essential workflows (Windows PowerShell)
- Build the project: `.
  \gradlew.bat build` (use the Gradle wrapper from project root).
- Run tests: `.
  \gradlew.bat test`.
- Deploy to RoboRIO: `.
  \gradlew.bat deploy` (networked RoboRIO required; team number comes from WPILib prefs or CLI).
- Simulator / WPILib sim tasks: inspect available tasks first with `.
  \gradlew.bat tasks --all` and run the appropriate WPILib sim task (this project enables the sim GUI in `build.gradle`).

Project-specific patterns and conventions
- Constants-first: Hardware IDs, limits and pipeline indexes live in `Constants.java`. When changing anything hardware-related (motor IDs, DIO channels, camera names), update `Constants` first and search usages.
- Naming convention: constants use `k` prefix (e.g. `kLeaderMotorID`, `kIntakeMotorID`) and nested classes for subsystem grouping.
- Vision: This repo uses a dual-Limelight setup. Camera names and pipeline indices are declared in `VisionConstants` (`kFixedCameraName`, `PIPELINE_TAGS`, etc.). If updating Limelight settings, ensure names match the Limelight web UI.
- Deploy files: Static files used at runtime (PathPlanner files, config JSON) are in `src/main/deploy/` (see `src/main/deploy/pathplanner/*`). When adding runtime assets, add them here so Gradle's `deploy` task copies them to `/home/lvuser/deploy` on the robot.
- Generated code: There is a `generated` package and compiled generated classes in `build/` — treat those as generated artifacts (don't hand-edit generated sources).

Integration points / vendor deps
- Vendor dependency manifests live under `vendordeps/` (e.g. `Phoenix6-26.1.0.json`, `REVLib.json`, `PathplannerLib-2026.1.2.json`). Use these when tracing JNI/native bindings or vendor-specific behavior.
- Notable integrations: WPILib (core), CTRE/REV vendor libraries, PathPlanner (paths under `src/main/deploy/pathplanner/`), Photon/vision libraries referenced via vendordeps.
- The build manifest (`build.gradle`) sets `ROBOT_MAIN_CLASS = "frc.robot.Main"` — if moving the robot entrypoint adjust the manifest there.

Examples (how to make common edits)
- Add a motor ID: update `Constants.ElevatorConstants.kLeaderMotorID` and then update any `new TalonFX(Constants.ElevatorConstants.kLeaderMotorID)` usages.
- Change Limelight pipeline index: update `Constants.VisionConstants.PIPELINE_TAGS` and ensure callers use `VisionConstants.PIPELINE_TAGS` instead of magic numbers.
- Add a PathPlanner path: put `.path` files in `src/main/deploy/pathplanner/paths/` and confirm `deploy` includes them (the Gradle deploy config includes `src/main/deploy`).

Quick tips for the assistant
- Prefer editing project source under `src/main/java/frc/robot/` and `src/main/deploy/` only. Avoid touching `build.gradle` unless required for dependency additions.
- When introducing new configuration keys, add them to the appropriate nested class in `Constants.java` rather than creating ad-hoc static fields elsewhere.
- Use the Gradle wrapper (`gradlew.bat`) for any build-related actions; do not assume global Gradle is available.
- To discover runtime tasks (simulator, deploy), run `.
  \gradlew.bat tasks --all` and select the WPILib tasks — the project enables the sim GUI (`wpi.sim.addGui()`) in `build.gradle`.

Where to look first (quick navigation)
- Robot code root: `src/main/java/frc/robot/`
- Constants: `src/main/java/frc/robot/Constants.java` (single source of truth for hardware IDs and pipeline indexes)
- Deploy assets & PathPlanner: `src/main/deploy/pathplanner/`
- Gradle config: `build.gradle`, `settings.gradle` (WPILib/GradleRIO configuration)
- Vendor manifests: `vendordeps/` (trace native/JNI bindings and versions)

If anything here is unclear or you want additions (e.g., common PR/branch rules, preferred code style, tests to run), tell me what to include and I will iterate.
