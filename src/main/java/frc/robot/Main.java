// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Main class is the entry point for the 2026 FRC robot program.
 * 
 * <p>This class should generally not be modified. It performs the low-level 
 * initialization required to start the robot code and link it to the 
 * WPILib {@link Robot} class.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any complex logic here.
   * 
   * @param args Command line arguments passed by the RoboRIO environment.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
