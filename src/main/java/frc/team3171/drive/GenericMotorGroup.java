package frc.team3171.drive;

// FRC Imports
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

// CTRE Imports
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

/**
 * @author Mark Ebert
 */
public class GenericMotorGroup implements MotorGroup {

    public enum MotorType {
        TalonFX, TalonSRX
    }

    // Variables
    private final boolean followSupport;
    private final MotorType motorType;

    // Motor Controllers
    private final MotorController masterMotor;
    private final MotorController[] slaveMotors;

    /**
     * Constructor
     * 
     * @param inverted         Whether or not the direction of the motors in the
     *                         {@link MotorController} need to be inverted.
     * @param motorController  The first {@link MotorController} to be used in
     *                         the {@link MotorGroup}.
     * @param motorControllers The rest of the {@link MotorController} to be used in
     *                         the {@link MotorGroup}.
     * @throws Exception Throws a new exception if there are an invalid amount of
     *                   motors.
     */
    public GenericMotorGroup(final boolean inverted, final MotorType motorType,
            final int motorController,
            final int... motorControllers) throws Exception {

        slaveMotors = new MotorController[motorControllers.length];

        // Sets whether or not to enable slave motors using following support
        this.motorType = motorType;
        switch (motorType) {
            case TalonFX:
                // Init the master motor
                masterMotor = new FRCTalonFX(motorController);
                masterMotor.setInverted(inverted);
                for (int i = 0; i < motorControllers.length; i++) {
                    slaveMotors[i] = new FRCTalonFX(motorControllers[i]);
                    ((BaseMotorController) slaveMotors[i]).follow((BaseMotorController) masterMotor);
                    ((BaseMotorController) slaveMotors[i]).setInverted(InvertType.FollowMaster);
                }
                followSupport = true;
                break;
            case TalonSRX:
                // Init the master motor
                masterMotor = new FRCTalonSRX(motorController);
                masterMotor.setInverted(inverted);
                for (int i = 0; i < motorControllers.length; i++) {
                    slaveMotors[i] = new FRCTalonSRX(motorControllers[i]);
                    ((BaseMotorController) slaveMotors[i]).follow((BaseMotorController) masterMotor);
                    ((BaseMotorController) slaveMotors[i]).setInverted(InvertType.FollowMaster);
                }
                followSupport = true;
                break;
            default:
                followSupport = false;
                throw new Exception("Invalid type of motor provided to the motor group!");
        }
    }

    /**
     * Constructor
     * 
     * @param inverted         Whether or not the direction of the motors in the
     *                         {@link MotorController} need to be inverted.
     * @param motorController  The first {@link MotorController} to be used in
     *                         the {@link MotorGroup}.
     * @param motorControllers The rest of the {@link MotorController} to be used in
     *                         the {@link MotorGroup}.
     * @throws Exception Throws a new exception if there are an invalid amount of
     *                   motors.
     */
    public GenericMotorGroup(final boolean inverted, final MotorType motorType,
            final int... motorControllers) throws Exception {
        if (motorControllers.length < 2) {
            throw new Exception("Invalid amount of motors provided!! At least two motors are needed.");
        }

        slaveMotors = new MotorController[motorControllers.length - 1];

        // Sets whether or not to enable slave motors using following support
        this.motorType = motorType;
        switch (motorType) {
            case TalonFX:
                // Init the master motor
                masterMotor = new FRCTalonFX(motorControllers[0]);
                masterMotor.setInverted(inverted);
                for (int i = 0; i < motorControllers.length - 1; i++) {
                    slaveMotors[i] = new FRCTalonFX(motorControllers[i + 1]);
                    ((BaseMotorController) slaveMotors[i]).follow((BaseMotorController) masterMotor);
                    ((BaseMotorController) slaveMotors[i]).setInverted(InvertType.FollowMaster);
                }
                followSupport = true;
                break;
            case TalonSRX:
                // Init the master motor
                masterMotor = new FRCTalonSRX(motorControllers[0]);
                masterMotor.setInverted(inverted);
                for (int i = 0; i < motorControllers.length - 1; i++) {
                    slaveMotors[i] = new FRCTalonSRX(motorControllers[i + 1]);
                    ((BaseMotorController) slaveMotors[i]).follow((BaseMotorController) masterMotor);
                    ((BaseMotorController) slaveMotors[i]).setInverted(InvertType.FollowMaster);
                }
                followSupport = true;
                break;
            default:
                followSupport = false;
                throw new Exception("Invalid type of motor provided to the motor group!");
        }
    }

    /**
     * Sets all of the {@link TalonFX} motors in the {@linkplain TalonFXMotorGroup}
     * to the desired speed.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the {@linkplain TalonFX}
     *              motors to.
     */
    @Override
    public void set(final double speed) {
        /*
         * Sets the speed of the master TalonFX, and therefore it's followers, to the
         * given value
         */
        masterMotor.set(speed);
        if (!followSupport) {
            for (MotorController slaveMotor : slaveMotors) {
                slaveMotor.set(speed);
            }
        }
    }

    /**
     * Sets whether or not the direction of the {@link TalonFX} motors in the
     * {@linkplain TalonFXMotorGroup} need to be inverted.
     * 
     * @param inverted Whether or not the direction of the {@link TalonFX} motors
     *                 need to be inverted.
     */
    @Override
    public void setInverted(final boolean inverted) {
        /*
         * Sets whether or not the direction of the master TalonFX, and therefore it's
         * followers, need to be inverted
         */
        masterMotor.setInverted(inverted);
        if (!followSupport) {
            for (MotorController slaveMotor : slaveMotors) {
                slaveMotor.setInverted(inverted);
            }
        }
    }

    /**
     * Gets whether or not the direction of the {@link TalonFX} motors in the
     * {@linkplain TalonFXMotorGroup} are inverted.
     * 
     * @return True, if the motors are inverted, false otherwise.
     */
    @Override
    public boolean getInverted() {
        /*
         * Gets whether or not the direction of the master TalonFX, and therefore it's
         * followers, are inverted
         */
        return masterMotor.getInverted();
    }

    /**
     * Returns the raw value of the {@link TalonFX} integrated encoder. The encoder
     * has 2048 ticks per revolution.
     * 
     * @return The raw value of the {@link TalonFX} integrated encoder.
     */
    public int getEncoderValue() {
        switch (motorType) {
            case TalonFX:
                return (int) ((BaseMotorController) masterMotor).getSelectedSensorPosition();
            case TalonSRX:
            default:
                return 0;
        }
    }

    /**
     * Returns the velocity of the {@link TalonFX} integrated encoder. The encoder
     * has 2048 ticks per revolution and the return units of the velocity is in
     * ticks per 100ms.
     * 
     * @return The velocity, in ticks per 100ms, of the {@link TalonFX} integrated
     *         encoder.
     */
    public int getEncoderVelocity() {
        switch (motorType) {
            case TalonFX:
                return (int) ((BaseMotorController) masterMotor).getSelectedSensorVelocity();
            case TalonSRX:
            default:
                return 0;
        }
    }

    /**
     * Disables all of the {@link TalonFX} motors in the
     * {@linkplain TalonFXMotorGroup}.
     */
    @Override
    public void disable() {
        /*
         * Sets the speed of the master TalonFX, and therefore it's followers, to the
         * value of 0 and disables it
         */
        masterMotor.disable();
        if (!followSupport) {
            for (MotorController slaveMotor : slaveMotors) {
                slaveMotor.disable();
            }
        }
    }

}