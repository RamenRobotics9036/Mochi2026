//LimelightHelpers v1.10 (REQUIRES LLOS 2024.9.1 OR LATER)

package frc.robot;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
//import frc.robot.LimelightHelpers.LimelightResults;
//import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.Map;
import java.util.concurrent.CompletableFuture;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.concurrent.ConcurrentHashMap;

/**
 * LimelightHelpers provides static methods and classes for interfacing with Limelight vision cameras in FRC.
 * This library supports all Limelight features including AprilTag tracking, Neural Networks, and standard color/retroreflective tracking.
 */
public class LimelightHelpers {

    private static final Map<String, DoubleArrayEntry> doubleArrayEntries = new ConcurrentHashMap<>();

    /**
     * Represents a Color/Retroreflective Target Result extracted from JSON Output
     */
    public static class LimelightTarget_Retro {

        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        private  double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace()
        {
            return toPose3D(cameraPose_TargetSpace);
        }
        public Pose3d getRobotPose_FieldSpace()
        {
            return toPose3D(robotPose_FieldSpace);
        }
        public Pose3d getRobotPose_TargetSpace()
        {
            return toPose3D(robotPose_TargetSpace);
        }
        public Pose3d getTargetPose_CameraSpace()
        {
            return toPose3D(targetPose_CameraSpace);
        }
        public Pose3d getTargetPose_RobotSpace()
        {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D()
        {
            return toPose2D(cameraPose_TargetSpace);
        }
        public Pose2d getRobotPose_FieldSpace2D()
        {
            return toPose2D(robotPose_FieldSpace);
        }
        public Pose2d getRobotPose_TargetSpace2D()
        {
            return toPose2D(robotPose_TargetSpace);
        }
        public Pose2d getTargetPose_CameraSpace2D()
        {
            return toPose2D(targetPose_CameraSpace);
        }
        public Pose2d getTargetPose_RobotSpace2D()
        {
            return toPose2D(targetPose_RobotSpace);
        }

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;
        
        @JsonProperty("ty")
        public double ty;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        @JsonProperty("ts")
        public double ts;

        public LimelightTarget_Retro() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }

    }

    /**
     * Represents an AprilTag/Fiducial Target Result extracted from JSON Output
     */
    public static class LimelightTarget_Fiducial {

        @JsonProperty("fID")
        public double fiducialID;

        @JsonProperty("fam")
        public String fiducialFamily;

        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace()
        {
            return toPose3D(cameraPose_TargetSpace);
        }
        public Pose3d getRobotPose_FieldSpace()
        {
            return toPose3D(robotPose_FieldSpace);
        }
        public Pose3d getRobotPose_TargetSpace()
        {
            return toPose3D(robotPose_TargetSpace);
        }
        public Pose3d getTargetPose_CameraSpace()
        {
            return toPose3D(targetPose_CameraSpace);
        }
        public Pose3d getTargetPose_RobotSpace()
        {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D()
        {
            return toPose2D(cameraPose_TargetSpace);
        }
        public Pose2d getRobotPose_FieldSpace2D()
        {
            return toPose2D(robotPose_FieldSpace);
        }
        public Pose2d getRobotPose_TargetSpace2D()
        {
            return toPose2D(robotPose_TargetSpace);
        }
        public Pose2d getTargetPose_CameraSpace2D()
        {
            return toPose2D(targetPose_CameraSpace);
        }
        public Pose2d getTargetPose_RobotSpace2D()
        {
            return toPose2D(targetPose_RobotSpace);
        }
        
        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        @JsonProperty("ts")
        public double ts;
        
        public LimelightTarget_Fiducial() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }
    }

    /**
     * Represents a Barcode Target Result extracted from JSON Output
     */
    public static class LimelightTarget_Barcode {

        /**
         * Barcode family type (e.g. "QR", "DataMatrix", etc.)
         */
        @JsonProperty("fam")
        public String family;

        /**
         * Gets the decoded data content of the barcode
         */
        @JsonProperty("data") 
        public String data;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("pts")
        public double[][] corners;

        public LimelightTarget_Barcode() {
        }

        public String getFamily() {
            return family;
        }
    }

    /**
     * Represents a Neural Classifier Pipeline Result extracted from JSON Output
     */
    public static class LimelightTarget_Classifier {

        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public double classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("zone")
        public double zone;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        public  LimelightTarget_Classifier() {
        }
    }

    /**
     * Represents a Neural Detector Pipeline Result extracted from JSON Output
     */
    public static class LimelightTarget_Detector {

        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public double classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        public LimelightTarget_Detector() {
        }
    }

    /**
     * Represents a 3D Pose Estimate.
     */
    public static class PoseEstimate {
        public Pose2d pose;
        public double timestampSeconds;
        public double latency;
        public int tagCount;
        public double tagSpan;
        public double avgTagDist;
        public double avgTagArea;

        public RawFiducial[] rawFiducials; 
        public boolean isMegaTag2;

        public PoseEstimate() {
            this.pose = new Pose2d();
            this.timestampSeconds = 0;
            this.latency = 0;
            this.tagCount = 0;
            this.tagSpan = 0;
            this.avgTagDist = 0;
            this.avgTagArea = 0;
            this.rawFiducials = new RawFiducial[]{};
            this.isMegaTag2 = false;
        }

        public PoseEstimate(Pose2d pose, double timestampSeconds, double latency, 
            int tagCount, double tagSpan, double avgTagDist, 
            double avgTagArea, RawFiducial[] rawFiducials, boolean isMegaTag2) {

            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.latency = latency;
            this.tagCount = tagCount;
            this.tagSpan = tagSpan;
            this.avgTagDist = avgTagDist;
            this.avgTagArea = avgTagArea;
            this.rawFiducials = rawFiducials;
            this.isMegaTag2 = isMegaTag2;
        }
    }

    /**
     * Represents a Limelight Raw Fiducial result from Limelight's NetworkTables output.
     */
    public static class RawFiducial {
        public int id = 0;
        public double txnc = 0;
        public double tync = 0;
        public double ta = 0;
        public double distToCamera = 0;
        public double distToRobot = 0;
        public double ambiguity = 0;

        public RawFiducial(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot, double ambiguity) {
            this.id = id;
            this.txnc = txnc;
            this.tync = tync;
            this.ta = ta;
            this.distToCamera = distToCamera;
            this.distToRobot = distToRobot;
            this.ambiguity = ambiguity;
        }
    }

    private static ObjectMapper mapper;

    /**
     * Print JSON Parse time to the console in milliseconds
     */
    static boolean profileJSON = false;

    static final String sanitizeName(String name) {
        if (name == null || name.equals("")) {
            return "limelight";
        }
        return name;
    }

    public static Pose3d toPose3D(double[] inData){
        if(inData.length < 6)
        {
            //System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }
        return new Pose3d(
            new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                    Units.degreesToRadians(inData[5])));
    }

    public static Pose2d toPose2D(double[] inData){
        if(inData.length < 6)
        {
            //System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    private static double extractArrayEntry(double[] inData, int position){
        if(inData.length < position+1)
        {
            return 0;
        }
        return inData[position];
    }

    private static PoseEstimate getBotPoseEstimate(String limelightName, String entryName, boolean isMegaTag2) {
        DoubleArrayEntry poseEntry = LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, entryName);
        
        TimestampedDoubleArray tsValue = poseEntry.getAtomic();
        double[] poseArray = tsValue.value;
        long timestamp = tsValue.timestamp;
        
        if (poseArray.length == 0) {
            // Handle the case where no data is available
            return new PoseEstimate();
        }
    
        var pose = toPose2D(poseArray);
        double latency = extractArrayEntry(poseArray, 6);
        int tagCount = (int)extractArrayEntry(poseArray, 7);
        double tagSpan = extractArrayEntry(poseArray, 8);
        double tagDist = extractArrayEntry(poseArray, 9);
        double tagArea = extractArrayEntry(poseArray, 10);
        
        // Convert server timestamp from microseconds to seconds and adjust for latency
        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);
    
        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;
        RawFiducial[] rawFiducials;

        if (poseArray.length != expectedTotalVals) {
            // Array size mismatch - return empty array instead of null-filled array
            rawFiducials = new RawFiducial[0];
        } else {
            rawFiducials = new RawFiducial[tagCount];
            for(int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = (int)poseArray[baseIndex];
                double txnc = poseArray[baseIndex + 1];
                double tync = poseArray[baseIndex + 2];
                double ta = poseArray[baseIndex + 3];
                double distToCamera = poseArray[baseIndex + 4];
                double distToRobot = poseArray[baseIndex + 5];
                double ambiguity = poseArray[baseIndex + 6];
                rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }
    
        return new PoseEstimate(pose, adjustedTimestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials, isMegaTag2);
    }

    public static RawFiducial[] getRawFiducials(String limelightName) {
        var entry = LimelightHelpers.getLimelightNTTableEntry(limelightName, "rawfiducials");
        var rawFiducialArray = entry.getDoubleArray(new double[0]);
        int valsPerEntry = 7;
        if (rawFiducialArray.length % valsPerEntry != 0) {
            return new RawFiducial[0];
        }
    
        int numFiducials = rawFiducialArray.length / valsPerEntry;
        RawFiducial[] rawFiducials = new RawFiducial[numFiducials];
    
        for (int i = 0; i < numFiducials; i++) {
            int baseIndex = i * valsPerEntry;
            int id = (int) extractArrayEntry(rawFiducialArray, baseIndex);
            double txnc = extractArrayEntry(rawFiducialArray, baseIndex + 1);
            double tync = extractArrayEntry(rawFiducialArray, baseIndex + 2);
            double ta = extractArrayEntry(rawFiducialArray, baseIndex + 3);
            double distToCamera = extractArrayEntry(rawFiducialArray, baseIndex + 4);
            double distToRobot = extractArrayEntry(rawFiducialArray, baseIndex + 5);
            double ambiguity = extractArrayEntry(rawFiducialArray, baseIndex + 6);
            
            rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
        }
    
        return rawFiducials;
    }

    public static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    public static void Flush() {
        NetworkTableInstance.getDefault().flush();
    }

    public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
        return getLimelightNTTable(tableName).getEntry(entryName);
    }

    public static DoubleArrayEntry getLimelightDoubleArrayEntry(String tableName, String entryName) {
        String key = tableName + "/" + entryName;
        return doubleArrayEntries.computeIfAbsent(key, k -> {
            NetworkTable table = getLimelightNTTable(tableName);
            return table.getDoubleArrayTopic(entryName).getEntry(new double[0]);
        });
    }
    
    public static double getLimelightNTDouble(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
    }

    public static void setLimelightNTDouble(String tableName, String entryName, double val) {
        getLimelightNTTableEntry(tableName, entryName).setDouble(val);
    }

    public static void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
        getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
    }

    public static double[] getLimelightNTDoubleArray(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
    }


    public static String getLimelightNTString(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getString("");
    }

    /**
     * Gets the current target count
     * @param limelightName
     * @return
     */
    public static int getTargetCount(String limelightName) {
        double[] t2d = getT2DArray(limelightName);
        if(t2d.length == 17)
        {
            return (int)t2d[1];
        }
        return 0;
    }

    public static double[] getT2DArray(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "t2d");
    }

    /////

    /**
     * Does the Limelight have a valid target?
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return True if a valid target is present, false otherwise
     */
    public static boolean getTV(String limelightName) {
        return 1.0 == getLimelightNTDouble(limelightName, "tv");
    }

    /**
     * Gets the horizontal offset from the crosshair to the target in degrees.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Horizontal offset angle in degrees
     */
    public static double getTX(String limelightName) {
        return getLimelightNTDouble(limelightName, "tx");
    }

    /**
     * Gets the vertical offset from the crosshair to the target in degrees.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Vertical offset angle in degrees
     */
    public static double getTY(String limelightName) {
        return getLimelightNTDouble(limelightName, "ty");
    }

    /**
     * Gets the target area as a percentage of the image (0-100%).
     * @param limelightName Name of the Limelight camera ("" for default) 
     * @return Target area percentage (0-100)
     */
    public static double getTA(String limelightName) {
        return getLimelightNTDouble(limelightName, "ta");
    }

    /**
     * Gets the pipeline's processing latency contribution.
     * @param limelightName Name of the Limelight camera
     * @return Pipeline latency in milliseconds
     */
    public static double getLatency_Pipeline(String limelightName) {
        return getLimelightNTDouble(limelightName, "tl");
    }

    /**
     * Gets the capture latency.
     * @param limelightName Name of the Limelight camera
     * @return Capture latency in milliseconds
     */
    public static double getLatency_Capture(String limelightName) {
        return getLimelightNTDouble(limelightName, "cl");
    }

    /**
     * Gets the active pipeline index.
     * @param limelightName Name of the Limelight camera
     * @return Current pipeline index (0-9)
     */
    public static double getCurrentPipelineIndex(String limelightName) {
        return getLimelightNTDouble(limelightName, "getpipe");
    }

    /**
     * Gets the current pipeline type.
     * @param limelightName Name of the Limelight camera
     * @return Pipeline type string (e.g. "retro", "apriltag", etc)
     */
    public static String getCurrentPipelineType(String limelightName) {
        return getLimelightNTString(limelightName, "getpipetype");
    }

    /**
     * Gets the full JSON results dump.
     * @param limelightName Name of the Limelight camera
     * @return JSON string containing all current results
     */
    public static String getJSONDump(String limelightName) {
        return getLimelightNTString(limelightName, "json");
    }

    /**
     * Switch to getBotPose
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    /**
     * Switch to getBotPose_wpiRed
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose_wpiRed(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    /**
     * Switch to getBotPose_wpiBlue
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose_wpiBlue(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    public static double[] getBotPose(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    public static double[] getBotPose_wpiRed(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    public static double[] getBotPose_wpiBlue(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    public static double[] getBotPose_TargetSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
    }

    public static double[] getCameraPose_TargetSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
    }

    public static double[] getTargetPose_CameraSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
    }

    public static double[] getTargetPose_RobotSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
    }

    public static double getFiducialID(String limelightName) {
        return getLimelightNTDouble(limelightName, "tid");
    }

    /**
     * Gets the Limelight heartbeat value.
     * @param limelightName Name of the Limelight camera
     * @return Heartbeat value (non-zero when connected)
     */
    public static double getHeartbeat(String limelightName) {
        return getLimelightNTDouble(limelightName, "hb");
    }

    /////
    /////

    public static Pose3d getBotPose3d(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose");
        return toPose3D(poseArray);
    }

    /**
     * (Not Recommended) Gets the robot's 3D pose in the WPILib Red Alliance Coordinate System.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the robot's position and orientation in Red Alliance field space
     */
    public static Pose3d getBotPose3d_wpiRed(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpired");
        return toPose3D(poseArray);
    }

    /**
     * (Recommended) Gets the robot's 3D pose in the WPILib Blue Alliance Coordinate System.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the robot's position and orientation in Blue Alliance field space
     */
    public static Pose3d getBotPose3d_wpiBlue(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
        return toPose3D(poseArray);
    }

    /**
     * Gets the robot's 3D pose with respect to the currently tracked target's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the robot's position and orientation relative to the target
     */
    public static Pose3d getBotPose3d_TargetSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the camera's 3D pose with respect to the currently tracked target's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the camera's position and orientation relative to the target
     */
    public static Pose3d getCameraPose3d_TargetSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the target's 3D pose with respect to the camera's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the target's position and orientation relative to the camera
     */
    public static Pose3d getTargetPose3d_CameraSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the target's 3D pose with respect to the robot's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the target's position and orientation relative to the robot
     */
    public static Pose3d getTargetPose3d_RobotSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
        return toPose3D(poseArray);
    }

    /**
     * Sets the Limelight's LED mode.
     * @param limelightName Name of the Limelight camera
     * @param ledMode The desired LED mode (0=default, 1=off, 2=blink, 3=on)
     */
    public static void setLEDMode_ForceOn(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 3);
    }

    public static void setLEDMode_ForceOff(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 1);
    }

    public static void setLEDMode_ForceBlink(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 2);
    }

    /**
     * Sets the Limelight's camera mode (vision or driver).
     * @param limelightName Name of the Limelight camera
     * @param camMode The desired camera mode (0=vision processing, 1=driver camera)
     */
    public static void setCameraMode_Processor(String limelightName) {
        setLimelightNTDouble(limelightName, "camMode", 0);
    }
    public static void setCameraMode_Driver(String limelightName) {
        setLimelightNTDouble(limelightName, "camMode", 1);
    }

    /**
     * Sets the Limelight's pipeline index (0-9).
     * @param limelightName Name of the Limelight camera
     * @param pipelineIndex The desired pipeline index (0-9)
     */
    public static void setPipelineIndex(String limelightName, double pipelineIndex) {
        setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
    }

    /**
     * Sets the Limelight's stream mode.
     * @param limelightName Name of the Limelight camera
     * @param streamMode The desired stream mode. 0=Standard, 1=PiP Main, 2=PiP Secondary
     */
    public static void setStreamMode_Standard(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 0);
    }

    public static void setStreamMode_PipMain(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 1);
    }

    public static void setStreamMode_PipSecondary(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 2);
    }

    /**
     * Sets the Limelight's crop window. The crop window must fit entirely within the default crop window.
     * @param limelightName Name of the Limelight camera
     * @param cropXMin Minimum X value (-1 to 1)
     * @param cropXMax Maximum X value (-1 to 1)
     * @param cropYMin Minimum Y value (-1 to 1)
     * @param cropYMax Maximum Y value (-1 to 1)
     */
    public static void setCropWindow(String limelightName, double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
        double[] cropWindowData = new double[4];
        cropWindowData[0] = cropXMin;
        cropWindowData[1] = cropXMax;
        cropWindowData[2] = cropYMin;
        cropWindowData[3] = cropYMax;
        setLimelightNTDoubleArray(limelightName, "crop", cropWindowData);
    }

    /**
     * Sets the robot's current orientation for MegaTag2 pose estimation.
     * @param limelightName Name of the Limelight camera
     * @param robotYaw Current robot yaw in degrees
     * @param yawRate Current yaw rate in degrees per second
     * @param robotPitch Current robot pitch in degrees
     * @param pitchRate Current pitch rate in degrees per second
     * @param robotRoll Current robot roll in degrees
     * @param rollRate Current roll rate in degrees per second
     */
    public static void SetRobotOrientation(String limelightName, double robotYaw, double yawRate, 
            double robotPitch, double pitchRate, 
            double robotRoll, double rollRate) {
        SetRobotOrientation(limelightName, robotYaw, yawRate, robotPitch, pitchRate, robotRoll, rollRate, true);
    }

    public static void SetRobotOrientation(String limelightName, double robotYaw, double yawRate, 
            double robotPitch, double pitchRate, 
            double robotRoll, double rollRate, boolean useNetwork) {

        double[] orientData = new double[6];
        orientData[0] = robotYaw;
        orientData[1] = (double)(int)yawRate;
        orientData[2] = robotPitch;
        orientData[3] = (double)(int)pitchRate;
        orientData[4] = robotRoll;
        orientData[5] = (double)(int)rollRate;
        setLimelightNTDoubleArray(limelightName, "robot_orientation_set", orientData);
    }

    /**
     * Gets the botpose estimate from a Limelight's NetworkTables entry in the WPILib Blue coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return PoseEstimate object containing the pose and associated metadata
     */
    public static PoseEstimate getBotPoseEstimate_wpiBlue(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpiblue", false);
    }

    /**
     * Gets the botpose estimate from a Limelight's NetworkTables entry in the WPILub Blue coordinate system, using MegaTag2.
     * @param limelightName Name/identifier of the Limelight
     * @return PoseEstimate object containing the pose and associated metadata
     */
    public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_orb_wpiblue", true);
    }

    /**
     * Gets the botpose estimate from a Limelight's NetworkTables entry in the WPILib Red coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return PoseEstimate object containing the pose and associated metadata
     */
    public static PoseEstimate getBotPoseEstimate_wpiRed(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpired", false);
    }

    /**
     * Gets the botpose estimate from a Limelight's NetworkTables entry in the WPILib Red coordinate system, using MegaTag2.
     * @param limelightName Name/identifier of the Limelight
     * @return PoseEstimate object containing the pose and associated metadata
     */
    public static PoseEstimate getBotPoseEstimate_wpiRed_MegaTag2(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_orb_wpired", true);
    }

    /**
     * Gets the botpose estimate from a Limelight's NetworkTables entry.
     * @param limelightName Name/identifier of the Limelight
     * @return PoseEstimate object containing the pose and associated metadata
     */
    public static PoseEstimate getBotPoseEstimate(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose", false);
    }

    /**
     * Gets the botpose estimate from a Limelight's NetworkTables entry, using MegaTag2.
     * @param limelightName Name/identifier of the Limelight
     * @return PoseEstimate object containing the pose and associated metadata
     */
    public static PoseEstimate getBotPoseEstimate_MegaTag2(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_orb", true);
    }
}
