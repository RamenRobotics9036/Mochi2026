package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The 2017 turret shooter hood subsystem
 * @author Peter Dentch
 */
public class HoodSubsystem extends SubsystemBase {
    private static HoodSubsystem instance = null;

    private static Servo hoodServo;

    private static double startAngle;
    private static double endAngle;
    private static double currentAngle;
    private static double startTime;

    public static final double WPILIB_MIN_SERVO_ANGLE = 0.0; //degrees
    public static final double WPILIB_MAX_SERVO_ANGLE = 360.0; //degrees
    private static final double TIME_TO_SERVO_FULL_EXTENSION = 3.48; //Avg time to move from retract to extend
    private static final double PERCENT_PER_SECOND = 1.00 / TIME_TO_SERVO_FULL_EXTENSION;
    private static final double DEGREES_PER_SECOND = (WPILIB_MAX_SERVO_ANGLE - WPILIB_MIN_SERVO_ANGLE)
    		* PERCENT_PER_SECOND;

    private static final double HOOD_MAX_POSITION = 1.0; //percent servo travel to max hood position
    private static final double HOOD_MIN_POSITION = 0.0; //percent servo travel to min hood position

    //SERVO Parameters from https://s3.amazonaws.com/actuonix/Actuonix+L16+Datasheet.pdf
    private static final int MAX_SERVO_PWM = 2000; //microseconds
    private static final int MIN_SERVO_PWM = 1000; //microseconds
    private static final int SERVO_RANGE = MAX_SERVO_PWM - MIN_SERVO_PWM;
    private static final int CENTER_SERVO_PWM = 1500; //microseconds
    private static final int SERVO_DEADBAND = 0; //microseconds - no deadband

    // pwm values in ms for the max and min angles of the shooter hood
    private static final int HOOD_MAX_PWM = MIN_SERVO_PWM + (int)Math.round(SERVO_RANGE * HOOD_MAX_POSITION);
    private static final int HOOD_MIN_PWM = MIN_SERVO_PWM + (int)Math.round(SERVO_RANGE * HOOD_MIN_POSITION);

    /**
     * Default constructor for the subsystem
     */
    public HoodSubsystem(){
    	hoodServo = new Servo(Constants.ShooterConstants.kHoodPwmChannel);
    	hoodServo.setBoundsMicroseconds(HOOD_MAX_PWM, CENTER_SERVO_PWM + SERVO_DEADBAND,
    			CENTER_SERVO_PWM, CENTER_SERVO_PWM - SERVO_DEADBAND, HOOD_MIN_PWM);
    }

    /**
     * Returns the shooter hood singleton object
     * @return is the current shooter hood object
     */
    public static HoodSubsystem getInstance(){
    	if(instance == null)
    		instance = new HoodSubsystem();

    	return instance;
    }

    /**
     * Takes a given angle and rotates the servo motor to that angle
     * @param degrees the angle limited by the min and max values defined in RobotMap
     */
    public void setAngle(double degrees){
    	if(degrees <= WPILIB_MIN_SERVO_ANGLE)
		degrees = WPILIB_MIN_SERVO_ANGLE;

	if(degrees >= WPILIB_MAX_SERVO_ANGLE)
		degrees = WPILIB_MAX_SERVO_ANGLE;


    // $TODO - C.L. TEMPORARY extra protection to avoid servo damage while testing
    if (degrees < 10.0) {
        degrees = 10.0;
    }
    if (degrees > 170.0) {
        degrees = 170.0;
    }


	startAngle = hoodServo.getAngle();
	startTime = Timer.getFPGATimestamp();

    System.out.println("Setting angle to "+Double.toString(degrees)+" degrees");
	hoodServo.setAngle(degrees);
    }

    /**
     * Returns the current angle of the servo by taking the angle it was last set to
     * with the time before the movement begins and after that is called the current
     * time and angle the servo is moving to is taken and the current angle is estimated
     * @return the estimated current angle of the servo in degrees
     */
    public double getAngle(){

	return hoodServo.getAngle();

	//endAngle = hoodServo.getAngle();
//	double angleDifference = endAngle - startAngle;
//	double timeDifference = Timer.getFPGATimestamp() - startTime;
//
//	if(angleDifference > 0)
//		currentAngle = (startAngle + (timeDifference * DEGREES_PER_SECOND));
//	else if(angleDifference < 0)
//		currentAngle = (startAngle - (timeDifference * DEGREES_PER_SECOND));
//	else //angleDifference == 0
//		currentAngle = endAngle;
//
//	return currentAngle;
    }

    /**
     * Sets the default command of the subsystem
     */
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	//setDefaultCommand(new DriveHoodWithJoystick());
    }
}
