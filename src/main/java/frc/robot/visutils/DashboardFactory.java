// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visutils;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.botconfig.RobotIdentity;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TestSubsystems;


public class DashboardFactory {

    private DashboardFactory() {}

    public static void initDebugDashboard(
            BotConfigInterface configInterface,
            Field2d glassField,
            DriveAccuracyTester driveAccuracyTester,
            IntakeSubsystem intakeSubsystem,
            IndexerSubsystem indexerSubsystem,
            ShooterSubsystem shooterSubsystem,
            ArmSubsystem armSubsystem,
            ClimberSubsystem climberSubsystem) {

        SmartDashboard.putString(
            "MAC Address Name",
            RobotIdentity.getBotName());
        SmartDashboard.putString(
            "Robot Config",
            configInterface.getConfigName());

        SmartDashboard.putData("GlassField", glassField);

        // Add a button on dashboard to launch Accuracy Drive Test
        SmartDashboard.putData("Accuracy Drive Test", driveAccuracyTester.createTapeDropAutoCommand());

        SmartDashboard.putData("Test Subsystems", TestSubsystems.test(
            intakeSubsystem,
            indexerSubsystem,
            shooterSubsystem,
            armSubsystem,
            climberSubsystem));
    }
}
