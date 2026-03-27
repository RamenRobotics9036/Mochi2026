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

### Old files we will replace/modify that *used* to handle Config

- `src/main/java/frc/robot/botconfig/BotConfigInterface.java` - Interface to query config through.
- `src/main/java/frc/robot/botconfig/CompConfig.java` - Implementation of BotConfigInterface for competition robot.
- `src/main/java/frc/robot/botconfig/PancakeConfig.java` - Implementation of BotConfigInterface for pancake/practice robot.
- `src/main/java/frc/robot/botconfig/RobotIdentity.java` - Gets the correct robot config based on MAC address.
- `src/main/java/frc/robot/util/MACAddress.java` - Utility code to get robot MAC address.
