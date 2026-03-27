# Per Robot Config

## Current files involved

- `src/main/java/frc/robot/botconfig/BotConfigInterface.java` - Interface to query config through.
- `src/main/java/frc/robot/botconfig/CompConfig.java` - Implementation of BotConfigInterface for competition robot.
- `src/main/java/frc/robot/botconfig/PancakeConfig.java` - Implementation of BotConfigInterface for pancake/practice robot.
- `src/main/java/frc/robot/botconfig/RobotIdentity.java` - Gets the correct robot config based on MAC address.
- `src/main/java/frc/robot/util/MACAddress.java` - Utility code to get robot MAC address.

## Goal

Make the config + identity system reusable so another team can plug in their own
robot configs without editing your core robot code.

## Current state (baseline)

- `BotConfigInterface` is good: most robot-dependent values are behind one contract.
- `CompConfig` and `PancakeConfig` are concrete adapters.
- `RobotIdentity` is hardcoded to your robots (MAC constants + direct `new CompConfig()` / `new PancakeConfig()`).

This means other teams must fork/modify identity selection logic, even if they can implement the interface.

## High-level options

### Option 1: Keep interface, make identity table-driven (lowest risk)

What changes:

- Keep `BotConfigInterface` and concrete classes exactly as the extension point.
- Replace hardcoded `if/else` in `RobotIdentity` with a registration table.
- Add a simple registration API so each team lists identity rules in one place.

Example concept:

- `RobotIdentity.register("Competition", new CompConfig(), mac(0x38, 0xD2, 0x58))`
- `RobotIdentity.register("Practice", new PancakeConfig(), mac(0x38, 0xD9, 0x80))`
- Optional fallback per runtime mode (`SIM`, `REAL_UNKNOWN`).

Pros:

- Smallest code churn.
- Easy migration from current design.
- Keeps compile-time safety and explicit code ownership.

Cons:

- Still code-based registration (not purely declarative).

Best for:

- Your immediate next step.

### Option 2: Split config contract into composable subcontracts (mid complexity)

What changes:

- Replace one broad interface with grouped interfaces:
	- `DrivetrainConfig`
	- `VisionConfig`
	- `FeatureToggleConfig`
	- `IdentityMetadata`
- `BotConfig` becomes an aggregate containing those pieces.

Pros:

- Clear ownership boundaries.
- Easier for teams to implement only relevant parts.
- Better long-term maintainability as features grow.

Cons:

- Moderate refactor across call sites.
- More types to maintain.

Best for:

- Teams expecting frequent architecture evolution.

### Option 3: Data-driven identity + config loading (JSON/YAML + adapters)

What changes:

- Move mostly-static values (toggles, camera transforms, names, speed limits) to JSON/YAML.
- Keep runtime/vendor-specific objects (e.g. Phoenix `SwerveModuleConstants`) in Java adapters.
- Identity mapping file controls which config profile applies for each MAC/hostname.

Pros:

- No code change needed for many tuning updates.
- Easier for external teams to onboard.
- Enables tool-driven config validation.

Cons:

- Requires schema/versioning/validation discipline.
- Not all config can be cleanly data-only because of generated CTRE types.

Best for:

- Multi-robot programs that tune frequently between events.

### Option 4: Provider/plugin model (highest flexibility)

What changes:

- Define provider interface, e.g. `RobotConfigProvider`:
	- `boolean supports(IdentityProbe probe)`
	- `IdentityResult build(IdentityProbe probe)`
- `RobotIdentity` discovers providers (manual registry or `ServiceLoader`).
- Each team supplies its own provider module/jar.

Pros:

- Strongest reuse boundary for open-source use.
- Teams can ship config logic independently.

Cons:

- Most architectural complexity.
- Harder debugging and deployment discipline required.

Best for:

- A shared library intended for many external teams.

### Option 5: Generic Name-to-Config Helper (simple and reusable)

What changes:

- Add a reusable helper that accepts a dictionary/map from robot name to config object.
- Type the helper with generics so the config can derive from a broad interface:
	- Concept: `RobotConfigRegistry<T extends BaseRobotConfig>`
	- Holds: `Map<String, T>`
	- Provides: `getRequired(String robotName)`, `getOrDefault(String robotName, String defaultName)`
- `RobotIdentity` is responsible only for detecting robot name/identity key, then asks the helper for the config.

Java interface note:

- Yes, Java interfaces can extend interfaces.
- Example shape:
	- `interface BaseRobotConfig { ... }`
	- `interface SwerveRobotConfig extends BaseRobotConfig { ... }`
	- `interface VisionRobotConfig extends BaseRobotConfig { ... }`

Pros:

- Matches your request directly (dictionary + general interface inheritance).
- Easy for teams to plug in custom config implementations.
- Keeps identity detection and config data concerns separated.

Cons:

- Name-based lookup still needs a convention (exact strings, enum, or constants).
- You still need a small assembly point to register available configs.

Best for:

- Teams wanting a lightweight, explicit extension mechanism without full plugin complexity.
