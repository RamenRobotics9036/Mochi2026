package frc.robot.vision;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LiveTunable {
    private final DoublePublisher pub;
    private final DoubleSubscriber sub;
    private final double defaultValue;
    private double lastValue;

    public LiveTunable(String path, double defaultValue) {
        this.defaultValue = defaultValue;
        var topic = NetworkTableInstance.getDefault().getDoubleTopic(path);
        pub = topic.publish();
        sub = topic.subscribe(defaultValue);
        pub.set(defaultValue);
        lastValue = defaultValue;
    }

    public double get() {
        double val = sub.get();
        if (val != lastValue) {
            pub.set(val);
            lastValue = val;
        }
        return val;
    }

    public double getDefault() {
        return defaultValue;
    }
}
