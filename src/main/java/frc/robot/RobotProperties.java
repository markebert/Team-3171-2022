package frc.robot;

// FRC Imports
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

// CTRE Imports
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * @author Mark Ebert
 */
public interface RobotProperties {

    /**
     * Static Values Properties
     * 
     * @param JOYSTICK_DEADZONE The percent of error allowed, from -1.0 to 1.0, of
     *                          the {@linkplain Joystick} X and Y values.
     * @param MAX_DRIVE_SPEED   The maximum drive speed of the robot, from 0 to 1.0
     */
    public static final double JOYSTICK_DEADZONE = .08;
    public static final double MAX_DRIVE_SPEED = .75;

    /**
     * CAN ID Properties
     * 
     * @param leftDriveCANIDArray   An int array representing the CAN IDs used for
     *                              the first left drive motors.
     * @param rightDriveCANIDArray  An int array representing the CAN IDs used for
     *                              the first right drive motors.
     * @param lowerShooterCANID     An int representing the CAN ID of the
     *                              {@linkplain TalonFX} motor.
     * @param upperShooterCANID     An int representing the CAN ID of the
     *                              {@linkplain TalonFX} motor.
     * @param pickupCANID           An int representing the CAN ID of the
     *                              {@linkplain TalonFX} motor.
     * @param lowerFeederCANIDArray An int array containing the CAN IDs of the
     *                              {@linkplain TalonSRX} motors
     * @param pcmCANID              An int representing the CAN ID of the PCM used
     *                              for both the
     *                              {@linkplain Compressor} motor and all of the
     *                              {@linkplain Solenoid} and
     *                              {@linkplain DoubleSolenoid}.
     */
    public static final int[] leftDriveCANIDArray = new int[] { 1, 2 };
    public static final int[] rightDriveCANIDArray = new int[] { 3, 4 };
    public static final int lowerShooterCANID = 10, upperShooterCANID = 11;
    public static final int pickupCANID = 5;
    public static final int upperFeederCANID = 12;
    public static final int[] lowerFeederCANIDArray = new int[] { 6, 7 };
    public static final int[] winchCANIDArray = new int[] { 8, 9 };
    public static final int pcmCANID = 14;
    /** End CAN ID Properties **/

    /**
     * Motor Inversion
     * 
     * @param shooterInverted     Whether or not the shooter motors need to be
     *                            inverted.
     * @param pickupInverted      Whether or not the pickup motor needs to be
     *                            inverted.
     * @param lowerFeederInverted Wether or not the feeder motor needs to be
     *                            inverted.
     */
    public static final boolean shooterInverted = true;
    public static final boolean pickupInverted = false;
    public static final boolean lowerFeederInverted = true;
    public static final boolean upperFeederInverted = true;
    public static final boolean pickupArmInverted = true;
    public static final boolean winchInverted = false;

    /** PID Properties **/
    /**
     * @param gyro_kP               The proportional value for the Gyro PID
     *                              Controller.
     * @param gyro_kI               The integral value for the Gyro PID Controller.
     * @param gyro_kD               The derivative value for the Gyro PID
     *                              Controller.
     * @param limelight_kP          The proportional value for the Limelight PID
     *                              Controller.
     * @param limelight_kI          The integral value for the Limelight PID
     *                              Controller.
     * @param limelight_kD          The derivative value for the Limelight PID
     *                              Controller.
     * @param shooter_kP            The proportional value for the Shooter PID
     *                              Controller.
     * @param shooter_kI            The integral value for the Shooter PID
     *                              Controller.
     * @param shooter_kD            The derivative value for the Shooter PID
     *                              Controller.
     * @param shooter_kF            The feed-forward value for the Shooter PID
     *                              Controller. kF: 1.0 represents output value to
     *                              TalonFX at 100%, 6380/600 represents Velocity
     *                              units per 100 ms at 100% output.
     * @param shooter_kPIDLoopIndex Sets whether or not the integrated PID
     *                              Controller of the Talon is closed loop (0) or
     *                              open loop (1).
     * @param shooter_kTimeoutMs    The timeout in ms to give the PID Controller of
     *                              the Talon.
     */
    public static final double gyro_kP = .0045, gyro_kI = .0001, gyro_kD = .0001;
    public static final double limelight_kP = .025, limelight_kI = .003, limelight_kD = .00175;
    public static final double shooter_kP = .01, shooter_kI = .0002, shooter_kD = .0001, shooter_kF = 0;
    public static final int shooter_kPIDLoopIndex = 0, shooter_kTimeoutMs = 20;
    /** End PID Properties **/

    /** Shooter Propterties **/
    /**
     * Shooter Controller CAN IDs
     * 
     * @param pickupArmForwardChannel
     * @param pickupArmReverseChannel
     * @param targetLightChannel      The DIO channel to use for the
     *                                {@linkplain DigitalOutput} to control the
     *                                targeting light relay.
     *                                <P>
     *                                0-9 are on-board, 10-25 are on the MXP.
     */
    public static final int pickupArmForwardChannel = 0;
    public static final int pickupArmReverseChannel = 1;
    public static final int targetLightChannel = 0;

    /** Auton Properties **/
    public static final String[] autonOptions = {
            "Auton 1",
            "Auton 2"
    };

}