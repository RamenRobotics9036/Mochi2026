# Per Robot Config

## What I implemented

I went with a generic `PerRobotConfig<T>` selector in `robot-utils` instead of
keeping team-specific robot identity logic inside the main robot project.

The core idea is:

- Teams define whatever config object or interface they want as `T`.
- The library does robot identification and config selection.
- Call sites get back the strongly typed config object they registered.

## Files involved

- `robot-utils/src/main/java/robotutils/perrobotconfig/PerRobotConfig.java`
- `robot-utils/src/main/java/robotutils/interfaces/PerRobotConfigInterface.java`
- `robot-utils/src/main/java/robotutils/interfaces/MacKey.java`
- `robot-utils/src/main/java/robotutils/perrobotconfig/MacAddress.java`
- `robot-utils/src/test/java/robotutils/perrobotconfig/TestPerRobotConfig.java`

## How it works

`PerRobotConfig<T>` is constructed from three maps plus two fallback config names:

- `Map<MacKey, String> macToRobotNameDict`
- `Map<String, String> robotNameToConfigNameDict`
- `Map<String, T> configNameToConfigObjDict`
- `String defaultConfigName`
- `String simulationConfigName`

Selection flow:

1. Match the current RoboRIO MAC address against `macToRobotNameDict`.
2. Convert the matched robot name into a config name with
   `robotNameToConfigNameDict`.
3. Return the config object from `configNameToConfigObjDict`.
4. If running in simulation, always use `simulationConfigName`.
5. If no MAC matches on real hardware, use `defaultConfigName`.

## Returned information

The shared interface exposes:

- `getRobotName()`
- `getBotConfig()`
- `getBotConfigName()`

Behavior:

- In simulation, `getRobotName()` returns `"Simulation"`.
- If no real robot matches, `getRobotName()` returns `"Unknown Robot"`.
- `getBotConfig()` returns the selected typed config object `T`.

## Validation and safety checks

The constructor validates that:

- all three maps are non-null and non-empty
- `defaultConfigName` exists in the config-object map
- `simulationConfigName` exists in the config-object map
- every robot name referenced by a MAC entry has a robot-to-config mapping
- every config name referenced by a robot has a config object

If any of those are invalid, it throws `IllegalArgumentException` with a direct
error message.

## Testing support

There is a second constructor used for tests that accepts:

- a `testMacKey` so tests do not need real hardware
- an optional forced simulation value

That lets the tests cover:

- input validation failures
- simulation override behavior
- matched MAC selection
- unknown robot fallback
- selected config object return value

## Why this was the right fit

This keeps the reusable library small and generic while still giving the robot
project strong typing and explicit control over its own config objects. It also
separates identity matching from the team-specific config implementation, which
was the main goal of the refactor.

## Switching to use this new config model

### Existing robot-side files involved

- `src/main/java/frc/robot/botconfig/BotConfigInterface.java`
- `src/main/java/frc/robot/botconfig/CompConfig.java`
- `src/main/java/frc/robot/botconfig/PancakeConfig.java`
- `src/main/java/frc/robot/botconfig/RobotIdentity.java`
- `src/main/java/frc/robot/util/MACAddress.java`
- `src/main/java/frc/robot/RobotContainer.java`
- `src/main/java/frc/robot/visutils/BasicInfoDashboard.java`

### Step-by-step migration plan

1. Keep `BotConfigInterface` as the team-facing config contract.
   `CompConfig` and `PancakeConfig` already implement the right shared type, so
   they can stay as-is and become the `T = BotConfigInterface` entries passed to
   `PerRobotConfig<BotConfigInterface>`.

2. Add a small robot-side builder/factory for `PerRobotConfig<BotConfigInterface>`.
   Create one place in the main robot project that constructs the selector using:
   - a `Map<MacKey, String>` for RoboRIO MAC suffix to robot name
   - a `Map<String, String>` for robot name to config name
   - a `Map<String, BotConfigInterface>` for config name to config object
   - the chosen default config name
   - the chosen simulation config name

   For this project, that mapping should represent the same choices currently
   hardcoded in `RobotIdentity`:
   - competition MAC `38:D2:58` -> `"Competition"` -> comp config
   - pancake MAC `38:D9:80` -> `"Pancake"` -> pancake config
   - simulation -> competition config
   - unknown real robot -> competition config

3. Replace `RobotIdentity.getBotConfig()` in `RobotContainer`.
   `RobotContainer` currently initializes `m_configInterface` directly from
   `RobotIdentity`. Change that so it holds a `PerRobotConfig<BotConfigInterface>`
   instance, then initialize:
   - `m_configInterface` from `perRobotConfig.getBotConfig()`
   - a robot-name field from `perRobotConfig.getRobotName()`
   - optionally a config-name field from `perRobotConfig.getBotConfigName()`

4. Replace `RobotIdentity.getBotName()` in `BasicInfoDashboard`.
   `BasicInfoDashboard` is the current direct consumer of the old robot-name API.
   Pass the selected robot name into the dashboard from `RobotContainer` instead
   of having the dashboard reach back into global identity logic.

5. Remove the old static identity singleton flow.
   Once the new `PerRobotConfig<BotConfigInterface>` object is created and owned
   by `RobotContainer` or a small factory class, `RobotIdentity` is no longer
   needed. Delete `src/main/java/frc/robot/botconfig/RobotIdentity.java` after
   all imports and call sites are switched over.

6. Remove the old robot-project MAC utility.
   After `RobotIdentity` is gone, the robot project should stop using
   `src/main/java/frc/robot/util/MACAddress.java`. The shared library already
   contains `robotutils.perrobotconfig.MacAddress`, so the old project-local
   copy should be deleted.

7. Verify there are no remaining references to the old model.
   Search for:
   - `RobotIdentity`
   - `frc.robot.util.MACAddress`
   - any direct import that assumed global static config lookup

   At the end of the migration, only the shared `PerRobotConfig` path should be
   responsible for robot identification.

8. Run focused tests on selection behavior.
   Confirm these cases still behave the same after the switchover:
   - competition RoboRIO selects `CompConfig`
   - pancake RoboRIO selects `PancakeConfig`
   - simulation selects the configured simulation config
   - unknown real hardware falls back to the configured default config
   - dashboard still shows the correct robot name and config name

### Expected end state

- `BotConfigInterface`, `CompConfig`, and `PancakeConfig` remain
- `PerRobotConfig<BotConfigInterface>` becomes the one config selector
- `RobotIdentity.java` is removed
- the old `src/main/java/frc/robot/util/MACAddress.java` is removed
- robot code gets config and robot name from the selected `PerRobotConfig`
  instance instead of from static global helpers
