package frc.team3171.drive;

// FRC Imports
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

// CTRE Imports
import com.ctre.phoenix.motorcontrol.InvertType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * @author Mark Ebert
 */
public class UniversalMotorGroup {

    // Supported motor types
    public enum ControllerType {
        TalonFX, TalonSRX, VictorSPX, VictorSP, CANSparkMaxBrushed, CANSparkMaxBrushless, PWMSparkMax
    }

    // Variables
    private final boolean followSupport;
    private final ControllerType controllerType;

    // Motor Controllers
    private final MotorController masterMotor;
    private final MotorController[] slaveMotors;

    /**
     * Constructor
     * 
     * @param inverted         Whether or not the direction of the motors in the
     *                         {@link MotorController} need to be inverted.
     * @param motorController  The first {@link MotorController} to be used in
     *                         the {@link UniversalMotorGroup}.
     * @param motorControllers The rest of the {@link MotorController} to be used in
     *                         the {@link UniversalMotorGroup}.
     * @throws Exception Throws a new exception if there are an invalid amount of
     *                   motors.
     */
    public UniversalMotorGroup(final boolean inverted, final ControllerType controllerType,
            final int motorController,
            final int... motorControllers) throws Exception {

        slaveMotors = new MotorController[motorControllers.length];

        // Sets whether or not to enable slave motors using following support
        this.controllerType = controllerType;
        switch (controllerType) {
            case TalonFX:
                // Init the master motor
                masterMotor = new FRCTalonFX(motorController);
                masterMotor.setInverted(inverted);
                for (int i = 0; i < slaveMotors.length; i++) {
                    slaveMotors[i] = new FRCTalonFX(motorControllers[i]);
                    ((FRCTalonFX) slaveMotors[i]).follow((FRCTalonFX) masterMotor);
                    ((FRCTalonFX) slaveMotors[i]).setInverted(InvertType.FollowMaster);
                }
                followSupport = true;
                break;
            case TalonSRX:
                // Init the master motor
                masterMotor = new FRCTalonSRX(motorController);
                masterMotor.setInverted(inverted);
                for (int i = 0; i < slaveMotors.length; i++) {
                    slaveMotors[i] = new FRCTalonSRX(motorControllers[i]);
                    ((FRCTalonSRX) slaveMotors[i]).follow((FRCTalonSRX) masterMotor);
                    ((FRCTalonSRX) slaveMotors[i]).setInverted(InvertType.FollowMaster);
                }
                followSupport = true;
                break;
            case VictorSPX:
                // Init the master motor
                masterMotor = new FRCVictorSPX(motorController);
                masterMotor.setInverted(inverted);
                for (int i = 0; i < slaveMotors.length; i++) {
                    slaveMotors[i] = new FRCVictorSPX(motorControllers[i]);
                    ((FRCVictorSPX) slaveMotors[i]).follow((FRCVictorSPX) masterMotor);
                    ((FRCVictorSPX) slaveMotors[i]).setInverted(InvertType.FollowMaster);
                }
                followSupport = true;
                break;
            case VictorSP:
                // Init the master motor
                masterMotor = new VictorSP(motorController);
                masterMotor.setInverted(inverted);
                for (int i = 0; i < slaveMotors.length; i++) {
                    slaveMotors[i] = new VictorSP(motorControllers[i]);
                    ((VictorSP) slaveMotors[i]).setInverted(inverted);
                }
                followSupport = false;
                break;
            case CANSparkMaxBrushed:
                // Init the master motor
                masterMotor = new CANSparkMax(motorController, MotorType.kBrushed);
                masterMotor.setInverted(inverted);
                ((CANSparkMax) masterMotor).setIdleMode(IdleMode.kBrake);
                for (int i = 0; i < slaveMotors.length; i++) {
                    slaveMotors[i] = new CANSparkMax(motorControllers[i], MotorType.kBrushed);
                    ((CANSparkMax) slaveMotors[i]).setIdleMode(IdleMode.kBrake);
                    ((CANSparkMax) slaveMotors[i]).follow((CANSparkMax) masterMotor);
                }
                followSupport = true;
                break;
            case CANSparkMaxBrushless:
                // Init the master motor
                masterMotor = new CANSparkMax(motorController, MotorType.kBrushless);
                masterMotor.setInverted(inverted);
                ((CANSparkMax) masterMotor).setIdleMode(IdleMode.kBrake);
                for (int i = 0; i < slaveMotors.length; i++) {
                    slaveMotors[i] = new CANSparkMax(motorControllers[i], MotorType.kBrushless);
                    ((CANSparkMax) slaveMotors[i]).setIdleMode(IdleMode.kBrake);
                    ((CANSparkMax) slaveMotors[i]).follow((CANSparkMax) masterMotor);
                }
                followSupport = true;
                break;
            case PWMSparkMax:
                // Init the master motor
                masterMotor = new PWMSparkMax(motorController);
                masterMotor.setInverted(inverted);
                for (int i = 0; i < slaveMotors.length; i++) {
                    slaveMotors[i] = new PWMSparkMax(motorControllers[i]);
                    ((PWMSparkMax) slaveMotors[i]).setInverted(inverted);
                }
                followSupport = false;
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
     *                         the {@link UniversalMotorGroup}.
     * @param motorControllers The rest of the {@link MotorController} to be used in
     *                         the {@link UniversalMotorGroup}.
     * @throws Exception Throws a new exception if there are an invalid amount of
     *                   motors.
     */
    public UniversalMotorGroup(final boolean inverted, final ControllerType motorType,
            final int... motorControllers) throws Exception {
        if (motorControllers.length < 2) {
            throw new Exception("Invalid amount of motors provided!! At least two motors are needed.");
        }

        slaveMotors = new MotorController[motorControllers.length - 1];

        // Sets whether or not to enable slave motors using following support
        this.controllerType = motorType;
        switch (motorType) {
            case TalonFX:
                // Init the master motor
                masterMotor = new FRCTalonFX(motorControllers[0]);
                masterMotor.setInverted(inverted);
                for (int i = 0; i < slaveMotors.length; i++) {
                    slaveMotors[i] = new FRCTalonFX(motorControllers[i + 1]);
                    ((FRCTalonFX) slaveMotors[i]).follow((FRCTalonFX) masterMotor);
                    ((FRCTalonFX) slaveMotors[i]).setInverted(InvertType.FollowMaster);
                }
                followSupport = true;
                break;
            case TalonSRX:
                // Init the master motor
                masterMotor = new FRCTalonSRX(motorControllers[0]);
                masterMotor.setInverted(inverted);
                for (int i = 0; i < slaveMotors.length; i++) {
                    slaveMotors[i] = new FRCTalonSRX(motorControllers[i + 1]);
                    ((FRCTalonSRX) slaveMotors[i]).follow((FRCTalonSRX) masterMotor);
                    ((FRCTalonSRX) slaveMotors[i]).setInverted(InvertType.FollowMaster);
                }
                followSupport = true;
                break;
            case VictorSPX:
                // Init the master motor
                masterMotor = new FRCVictorSPX(motorControllers[0]);
                masterMotor.setInverted(inverted);
                for (int i = 0; i < slaveMotors.length; i++) {
                    slaveMotors[i] = new FRCVictorSPX(motorControllers[i + 1]);
                    ((FRCVictorSPX) slaveMotors[i]).follow((FRCVictorSPX) masterMotor);
                    ((FRCVictorSPX) slaveMotors[i]).setInverted(InvertType.FollowMaster);
                }
                followSupport = true;
                break;
            case VictorSP:
                // Init the master motor
                masterMotor = new VictorSP(motorControllers[0]);
                masterMotor.setInverted(inverted);
                for (int i = 0; i < slaveMotors.length; i++) {
                    slaveMotors[i] = new VictorSP(motorControllers[i + 1]);
                    ((VictorSP) slaveMotors[i]).setInverted(inverted);
                }
                followSupport = false;
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
    public boolean getInverted() {
        /*
         * Gets whether or not the direction of the master TalonFX, and therefore it's
         * followers, are inverted
         */
        return masterMotor.getInverted();
    }

    /**
     * If the designated motor type is {@link ControllerType} {@link FRCTalonFX},
     * then
     * the function returns the raw value of the {@link FRCTalonFX} integrated
     * encoder, if not then 0 is always returned. The encoder has 2048 ticks per
     * revolution.
     * 
     * @return The raw value of the {@link FRCTalonFX} integrated encoder.
     */
    public int getEncoderValue() {
        switch (controllerType) {
            case TalonFX:
                return (int) ((FRCTalonFX) masterMotor).getSelectedSensorPosition();
            case TalonSRX:
            case VictorSPX:
            case VictorSP:
            default:
                return 0;
        }
    }

    /**
     * If the designated motor type is {@link ControllerType} {@link FRCTalonFX},
     * then
     * the function returns the velocity of the {@link FRCTalonFX} integrated
     * encoder, if not then 0 is always returned. The encoder has 2048 ticks per
     * revolution and the return units of the velocity is in ticks per 100ms.
     * 
     * @return The velocity, in ticks per 100ms, of the {@link FRCTalonFX}
     *         integrated encoder.
     */
    public int getEncoderVelocity() {
        switch (controllerType) {
            case TalonFX:
                return (int) ((FRCTalonFX) masterMotor).getSelectedSensorVelocity();
            case TalonSRX:
            case VictorSPX:
            case VictorSP:
            default:
                return 0;
        }
    }

    /**
     * Disables all of the {@link TalonFX} motors in the
     * {@linkplain TalonFXMotorGroup}.
     */
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