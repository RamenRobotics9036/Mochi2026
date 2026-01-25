# Remaining Cleanup for Sim

> **Goal:** Minimize changes required to add Simulation to the Ramen Robotics Mochi 2026 robot.

On the `users/ido/sim1` branch, diff commit `e17d69` ("Add sim files") to HEAD to see what changes were required in:

- `Robot.java`
- `RobotContainer.java`
- `CommandSwerveDrivetrain.java` (Drive subsystem)

---

## ✅ Required changes in Robot.java

Robot.java only requires creating `WrapperSimRobot` and calling the appropriate periodic methods.

---

## ❌ Required changes in RobotContainer.java
Possible improvements:
- **Joystick Input** - The way we fix Joystick input during simulation still seems complicated.
- **getGroundTruthSim()** - RobotContainer.java shouldnt every call getGroundTruthSim().  Instead, we should add a method injectDrift() to WrapperSimRobotContainer.
