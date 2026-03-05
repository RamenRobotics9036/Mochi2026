// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visutils;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.botconfig.RobotIdentity;
import frc.robot.sim.SimWrapper;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TestSubsystems;
import frc.robot.subsystems.auto.AutoLogic;


public class DashboardFactory {

    private DashboardFactory() {}

    public static void initDebugDashboard(
            BotConfigInterface configInterface,
            SimWrapper simWrapper,
            Field2d glassField,
            DriveAccuracyTester driveAccuracyTester,
            CommandSwerveDrivetrain drivetrain,
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

        // Add a button to reset the ground-truth pose to the selected auto starting pose
        if (simWrapper != null) {
            Command cycleResetCmd = drivetrain.runOnce(() ->
                simWrapper.cycleResetPosition(AutoLogic.getSelectedAutoStartingPose()));
            SmartDashboard.putData("Sim/CycleResetPosition", cycleResetCmd);
        }
    }
}
