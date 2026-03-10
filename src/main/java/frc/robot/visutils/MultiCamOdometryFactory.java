package frc.robot.visutils;

import frc.robot.botconfig.BotConfigInterface;
import frc.robot.botconfig.BotConfigInterface.CameraInfo;
import frc.robot.visutils.evaluateposes.EvaluatePosesFactory;
import frc.robot.visutils.evaluateposes.EvaluatePosesInterface;
import java.util.ArrayList;
import java.util.List;

/** Factory for creating and wiring up a {@link MultiCamOdometry} instance. */
public class MultiCamOdometryFactory {

    /** Constructor. */
    private MultiCamOdometryFactory() {}

    /**
     * Creates a {@link MultiCamOdometry} instance and wires it to the dashboard.
     *
     * @param configInterface    Bot configuration (cameras, speeds, etc.)
     * @param outputs            Camera outputs: pose consumer and Kalman filter
     * @param inputs             Camera inputs: drive state, pose sampler, and motionless supplier
     * @param basicInfoDashboard Dashboard to push vision state updates to each cycle
     * @param motionlessTracker  Tracks whether the robot is motionless
     * @param evaluatePosesName  Name of the {@link EvaluatePosesInterface} implementation
     *                           to use (e.g. {@code "MochiV1"})
     * @return A fully configured {@link MultiCamOdometryWrapper} instance
     */
    public static MultiCamOdometryWrapper create(
            BotConfigInterface configInterface,
            CamOutputs outputs,
            CamInputs inputs,
            BasicInfoDashboard basicInfoDashboard,
            MotionlessTracker motionlessTracker,
            String evaluatePosesName) {

        boolean megaTag2Enabled = configInterface.isMegaTag2Supported();
        boolean autoVisionInjectionEnabled = configInterface.isAutoVisionInjectionEnabled();
        EvaluatePosesInterface evaluatePoses = EvaluatePosesFactory.create(evaluatePosesName);
        CamConfig config = new CamConfig(
            megaTag2Enabled,
            autoVisionInjectionEnabled,
            evaluatePoses);

        List<CamOdometryInterface> cameras = new ArrayList<>();
        for (CameraInfo camInfo : configInterface.getCameras()) {
            cameras.add(new SingleCamOdometry(
                camInfo.cameraName,
                camInfo.robotToCam,
                outputs,
                inputs,
                config));
        }

        CamOdometryInterface multiCam = new MultiCamOdometry(
            cameras,
            megaTag2Enabled,
            autoVisionInjectionEnabled,
            config.evaluatePoses());

        MultiCamOdometryWrapper wrapper =
                new MultiCamOdometryWrapper(
                    multiCam,
                    configInterface.isVisionEnabledDefault(),
                    basicInfoDashboard,
                    motionlessTracker);

        return wrapper;
    }
}
