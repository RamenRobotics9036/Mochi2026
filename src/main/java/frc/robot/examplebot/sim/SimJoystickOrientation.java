package frc.robot.sim;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Helper class for handling joystick orientation in simulation.
 * In simulation, we ensure that "forward" is always towards the top of the screen,
 * regardless of alliance color.
 */
public class SimJoystickOrientation {

    public enum ScreenDirection { EAST, WEST }

    /**
     * Determines the operator's screen direction based on the operator forward angle.
     *
     * @param degrees The operator forward direction in degrees (from drivetrain.getOperatorForwardDirection())
     * @return The screen direction (EAST for blue alliance, WEST for red alliance)
     * @throws IllegalStateException if the degrees don't match expected alliance orientations
     */
    public static ScreenDirection getOperatorScreenDirection(double degrees) {
        if (degrees >= -45 && degrees < 45) {
            return ScreenDirection.EAST;  // Blue alliance: forward toward red wall
        } else if (degrees >= 135 || degrees < -135) {
            return ScreenDirection.WEST;  // Red alliance: forward toward blue wall
        } else {
            throw new IllegalStateException("Unexpected operator direction: " + degrees);
        }
    }

    /**
     * Applies joystick input to a field-centric drive request, adjusting for screen orientation.
     * This ensures "forward" on the joystick always moves the robot toward the top of the screen
     * in simulation, regardless of alliance color.
     *
     * @param drive The field-centric drive request to modify
     * @param degrees The operator forward direction in degrees
     * @param joystick The Xbox controller providing input
     * @param maxSpeed The maximum linear speed in meters per second
     * @param maxAngularRate The maximum angular rate in radians per second
     * @return The modified drive request with appropriate velocity mappings
     */
    public static SwerveRequest.FieldCentric applySimJoystickInput(
            SwerveRequest.FieldCentric drive,
            double degrees,
            CommandXboxController joystick,
            double maxSpeed,
            double maxAngularRate) {
        ScreenDirection direction = getOperatorScreenDirection(degrees);

        return switch (direction) {
            case EAST -> drive.withVelocityX(joystick.getLeftX() * maxSpeed)
                .withVelocityY(-joystick.getLeftY() * maxSpeed)
                .withRotationalRate(-joystick.getRightX() * maxAngularRate);
            case WEST -> drive.withVelocityX(-joystick.getLeftX() * maxSpeed)
                .withVelocityY(joystick.getLeftY() * maxSpeed)
                .withRotationalRate(-joystick.getRightX() * maxAngularRate);
        };
    }
}
