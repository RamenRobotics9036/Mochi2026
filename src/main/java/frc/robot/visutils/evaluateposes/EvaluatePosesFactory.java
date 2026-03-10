package frc.robot.visutils.evaluateposes;

/** Factory that returns the correct {@link EvaluatePosesInterface} implementation by name. */
public class EvaluatePosesFactory {

    private EvaluatePosesFactory() {}

    /**
     * Creates an {@link EvaluatePosesInterface} implementation by name.
     *
     * @param name Implementation name (e.g. {@code "MochiV1"}).
     * @return The requested implementation.
     * @throws IllegalArgumentException if {@code name} is not recognised.
     */
    public static EvaluatePosesInterface create(String name) {
        switch (name) {
            case "MochiV1":
                return new EvaluatePosesMochiV1();
            default:
                throw new IllegalArgumentException("Unknown EvaluatePoses implementation: " + name);
        }
    }
}
