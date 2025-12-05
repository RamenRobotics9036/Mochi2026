package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoNameConstants;

import java.io.IOException;
import java.util.List;
import org.json.simple.parser.ParseException;

@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class AutoLogic {
  public static SendableChooser<String> autoPicker = new SendableChooser<String>();

  public static ShuffleboardTab tab = Shuffleboard.getTab("Autos");

  // Constructor
  public AutoLogic() {
    throw new UnsupportedOperationException("This is a static utility class, dont try to instantiate!");
  }
  
  public static Command getAutoCommand(String autoName) {
    // System.out.println("Path name: " + pathName);
    // Load the path you want to follow using its name in the GUI
    try {

      return AutoBuilder.buildAuto(autoName);

    } catch (FileVersionException e) {
      // TODO: handle exception

      DriverStation.reportError("Ooofs: " + e.getMessage(), e.getStackTrace());
    }

    // Create a path following command using AutoBuilder. This will also trigger event markers.

    return Commands.none();
  }
  
  public static PathPlannerPath getPathData(String pathName) {
    // System.out.println("Path name: " + pathName);
    // Load the path you want to follow using its name in the GUI
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

      return path;

    } catch (IOException | ParseException | FileVersionException e) {
      // TODO: handle exception

      DriverStation.reportError("Ooofs: " + e.getMessage(), e.getStackTrace());
    }

    // Create a path following command using AutoBuilder. This will also trigger event markers.

    return null;
  }

  public static void initShuffleBoard() {
    // Leave these options available for kCompetitionMode

    addAutoOptions();

    tab.add("Auto Selector", autoPicker)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);

    tab.addString("Current Selected path", () -> autoPicker.getSelected());
    if (RobotState.isAutonomous()) {
      getAutoCommand(autoPicker.getSelected());
    }
  }

  public static <T> String[] toStringArray(List<T> dataList) {
   
    
    String[] data = new String[dataList.size()]; // TODO FIX INFINTELY REPEATING LIST

    for (int i = 0; i < dataList.size(); i++) {

      String addedData = dataList.get(i).toString();

      data[i] = "\n" + addedData;
    }

    return data;
  }

  public static PathPlannerTrajectory makeTrajectory(
      PathPlannerPath path,
      ChassisSpeeds startingSpeeds,
      Rotation2d startingRotation,
      RobotConfig config) {

    PathPlannerTrajectory trajectory =
        new PathPlannerTrajectory(path, startingSpeeds, startingRotation, config);

    return trajectory;
  }

  public static void addOptionToPicker(String autoName) {
    autoPicker.addOption(autoName, autoName);
  }

  public static void addOptionToPicker(String displayName, String autoName) {
    autoPicker.addOption(displayName, autoName);
  }

  public static String getSelectedName() {
    return autoPicker.getSelected();
  }

  public static boolean selectedAutoIsMirror() {
    return getSelectedName().startsWith("auto mirror");
  }

  public static void addAutoOptions() {
    //autoPicker.setDefaultOption("Right auto", "Birdi Auto Right");
    //autoPicker.setDefaultOption("Left Auto", "Birdi Auto left");
    autoPicker.setDefaultOption("Center Auto", "Birdi Auto Center");
    //autoPicker.setDefaultOption("test auto freedom project", "TTTTest");
    
    //This was used for the manual code of the Auto
    //autoPicker.setDefaultOption(AutoNameConstants.kCenterL1AutoName, AutoNameConstants.kCenterL1AutoName);
    //addOptionToPicker(AutoNameConstants.kCenterL4AutoccxName);
    
    //These are alternatives to choice if needed during the competition through PathPlanner
    addOptionToPicker("LEFT 1 Coral", "auto 1 coral left");
    addOptionToPicker("RIGHT 1 Coral", "auto 1 coral right");
  }
}