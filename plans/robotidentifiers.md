| Motor           | Device    | CAN ID | Position    | Function                | CAN bus sequence | PDH Port |
| --------------- | --------- | ------ | ----------- | ----------------------- | ---------------- | -------- |
| NA              | RoboRIO   | NA     | -----       | Command and control     | First            |          |
| NA              | CANCoder  | 0      | Front-Left  | Swerve Position         |                  | NA       |
| NA              | CANCoder  | 1      | Back-Left   | Swerve Position         |                  | NA       |
| NA              | CANCoder  | 2      | Back-Right  | Swerve Position         |                  | NA       |
| NA              | CANCoder  | 3      | Front-Right | Swerve Position         |                  | NA       |
| Kraken          | TalonFX   | 10     | Front-Left  | Rotation                |                  |          |
| Kraken          | TalonFX   | 11     | Front-Left  | Drive                   |                  |          |
| Kraken          | TalonFX   | 12     | Back-Left   | Rotation                |                  |          |
| Kraken          | TalonFX   | 13     | Back-Left   | Drive                   |                  |          |
| Kraken          | TalonFX   | 14     | Back-Right  | Rotation                |                  |          |
| Kraken          | TalonFX   | 15     | Back-Right  | Drive                   |                  |          |
| Kraken          | TalonFX   | 16     | Front-Right | Rotation                |                  |          |
| Kraken          | TalonFX   | 17     | Front-Right | Drive                   |                  |          |
|                 | Pigeon    | 7      | -----       | Inertial Refernece Unit |                  |          |
| Vortex          | SparkFLex | 20     | Left        | Intake arm              |                  |          |
| Vortex          | SparkFLex | 21     | Right       | Intake arm              |                  |          |
| Vortex          | SparkFLex | 22     | -----       | Roller                  |                  |          |
| Vortex          | SparkFLex | 30     | Left        | Indexer                 |                  |          |
| Kraken          | TalonFX   | 40     | Left        | Shooter                 |                  |          |
| Kraken          | TalonFX   | 41     | Right       | Shooter                 |                  |          |
| Votex           | SparkFlex | 50     | -----       | Climber                 |                  |          |
| Linear actuator |           |        | Right       | Hood                    |                  |          |
| NA              | PDH       | 1      | -----       | Power distribution      | Last             |          |
