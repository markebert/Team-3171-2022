package frc.team3171.controllers;

// Java Imports
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;

// FRC Imports
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;

// CTRE Imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.drive.FRCTalonFX;

/**
 * @author Mark Ebert
 */
public class Shooter implements RobotProperties {

    // Motor Controllers
    private final FRCTalonFX lowerShooterMotor, upperShooterMotor, pickupMotor, upperFeederMotor, lowerFeederMotor;
    private final CANSparkMax pickupArmMotor;
    private final RelativeEncoder pickupArmEncoder;
    private final SparkMaxPIDController pickupArmPIDController;

    // Executor Service
    private final ExecutorService executorService;

    // Reentrant Locks
    private final ReentrantLock executorLock;

    // Atomic Booleans
    private final AtomicBoolean lowerFeederExecutorActive, upperFeederExecutorActive;

    private int lowerVelocity, upperVelocity, desiredLowerVelocity, desiredUpperVelocity, pickupArmPosition;
    private double pickupArmCurrent;

    /**
     * Constructor
     * 
     * @throws Exception Throws a new exception if there are an invalid amount of
     *                   motors in the feederCANIDArray.
     */
    public Shooter() throws Exception {
        // Init all of the motors
        lowerShooterMotor = new FRCTalonFX(LOWER_SHOOTER_CAN_ID);
        upperShooterMotor = new FRCTalonFX(UPPER_SHOOTER_CAN_ID);
        pickupMotor = new FRCTalonFX(PICKUP_MOTOR_CAN_ID);
        upperFeederMotor = new FRCTalonFX(UPPER_FEEDER_CAN_ID);
        lowerFeederMotor = new FRCTalonFX(LOWER_FEEDER_CAN_ID);
        pickupArmMotor = new CANSparkMax(PICKUP_ARM_CAN_ID, MotorType.kBrushless);
        pickupArmMotor.setIdleMode(IdleMode.kBrake);

        // Set if any motors need to be inverted
        lowerShooterMotor.setInverted(LOWER_SHOOTER_INVERTED);
        upperShooterMotor.setInverted(UPPER_SHOOTER_INVERTED);
        pickupMotor.setInverted(PICKUP_MOTOR_INVERTED);
        upperFeederMotor.setInverted(UPPER_FEEDER_INVERTED);
        lowerFeederMotor.setInverted(LOWER_FEEDER_INVERTED);
        pickupArmMotor.setInverted(PICKUP_ARM_MOTOR_INVERTED);

        // Gets the Neo Spark Max encoder
        pickupArmEncoder = pickupArmMotor.getEncoder();
        pickupArmPIDController = pickupArmMotor.getPIDController();

        // set PID coefficients
        pickupArmPIDController.setP(PICKUP_ARM_KP);
        pickupArmPIDController.setI(PICKUP_ARM_KI);
        pickupArmPIDController.setD(PICKUP_ARM_KD);
        pickupArmPIDController.setFF(PICKUP_ARM_KF);
        pickupArmPIDController.setOutputRange(-1, 1);
        pickupArmPIDController.setSmartMotionMaxVelocity(2000, 0);
        pickupArmPIDController.setSmartMotionMinOutputVelocity(0, 0);
        pickupArmPIDController.setSmartMotionMaxAccel(1500, 0);
        pickupArmPIDController.setSmartMotionAllowedClosedLoopError(0, 0);

        // Configure the velocity closed loop values
        lowerShooterMotor.config_kP(0, SHOOTER_KP);
        lowerShooterMotor.config_kI(0, SHOOTER_KI);
        lowerShooterMotor.config_kD(0, SHOOTER_KD);
        lowerShooterMotor.config_kF(0, SHOOTER_KF);

        upperShooterMotor.config_kP(0, SHOOTER_KP);
        upperShooterMotor.config_kI(0, SHOOTER_KI);
        upperShooterMotor.config_kD(0, SHOOTER_KD);
        upperShooterMotor.config_kF(0, SHOOTER_KF);

        // Initialize the executor service for concurrency
        executorService = Executors.newFixedThreadPool(2);

        executorLock = new ReentrantLock(true);

        // Initialize the AtomicBooleans to control the thread executors
        lowerFeederExecutorActive = new AtomicBoolean(false);
        upperFeederExecutorActive = new AtomicBoolean(false);
    }

    public void init() {
        pickupArmMotor.setIdleMode(IdleMode.kCoast);
    }

    public void periodic() {
        lowerVelocity = (int) Math.round((lowerShooterMotor.getSelectedSensorVelocity() / 2048) * 600);
        upperVelocity = (int) Math.round((upperShooterMotor.getSelectedSensorVelocity() / 2048) * 600);
        desiredLowerVelocity = (int) Math.round((lowerShooterMotor.getClosedLoopTarget() / 2048) * 600);
        desiredUpperVelocity = (int) Math.round((upperShooterMotor.getClosedLoopTarget() / 2048) * 600);
        pickupArmPosition = (int) Math.round(pickupArmEncoder.getPosition());
        pickupArmCurrent = pickupArmMotor.getOutputCurrent();
    }

    /**
     * Sets the speed of the shooter motors to the given value.
     * 
     * @param lowerShooterSpeed The speed, from -1.0 to 1.0, to set the lower
     *                          shooter motor to.
     * @param upperShooterSpeed The speed, from -1.0 to 1.0, to set the upper
     *                          shooter motor to.
     */
    public void setShooterSpeed(final double lowerShooterSpeed, final double upperShooterSpeed) {
        lowerShooterMotor.set(ControlMode.PercentOutput, lowerShooterSpeed);
        upperShooterMotor.set(ControlMode.PercentOutput, upperShooterSpeed);
    }

    /**
     * Sets the speed of the shooter motors to the given value.
     * 
     * @param shooterSpeed The speed, from -1.0 to 1.0, to set the all of the
     *                     shooter motors to.
     */
    public void setShooterSpeed(final double shooterSpeed) {
        setShooterSpeed(shooterSpeed, shooterSpeed);
    }

    /**
     * Sets the RPM of the shooter motors to the given value.
     * 
     * @param lowerShooterRPM The RPM to set the lower shooter motor to.
     * @param upperShooterRPM The RPM to set the upper shooter motor to.
     */
    public void setShooterVelocity(final int lowerShooterRPM, final int upperShooterRPM) {
        /**
         * First check if either desired RPM is 0, if so lets the electronic brake
         * handle the slow done rather then the PID Controller.
         * <p>
         * Otherwise, converts the desired shooterRPM to units / 100ms.
         * <p>
         * (2048-Units/Rev * shooterRPM) / 600-100ms/min in either direction.
         * <p>
         * Velocity setpoint is in units/100ms.
         */
        if (lowerShooterRPM == 0) {
            lowerShooterMotor.set(ControlMode.PercentOutput, 0);
        } else {
            final double lowerTargetVelocity_UnitsPer100ms = (lowerShooterRPM * 2048.0) / 600;
            lowerShooterMotor.set(ControlMode.Velocity, lowerTargetVelocity_UnitsPer100ms);
        }
        if (upperShooterRPM == 0) {
            upperShooterMotor.set(ControlMode.PercentOutput, 0);
        } else {
            final double upperTargetVelocity_UnitsPer100ms = (upperShooterRPM * 2048.0) / 600;
            upperShooterMotor.set(ControlMode.Velocity, upperTargetVelocity_UnitsPer100ms);
        }
    }

    /**
     * Sets the RPM of the shooter motors to the given value.
     * 
     * @param shooterRPM The RPM to set the all of the shooter motors to.
     */
    public void setShooterVelocity(final int shooterRPM) {
        setShooterVelocity(shooterRPM, shooterRPM);
    }

    /**
     * Returns the velocity of the lower shooter motor in RPM, converted from Units
     * per 100ms.
     * 
     * @return The RPM of the lower shooter motor.
     */
    public double getLowerShooterVelocity() {
        return lowerVelocity;
    }

    /**
     * Returns if the velocity of the motor is within the provided percent error
     * margin.
     * 
     * @param percentError The percent error, from 0.0 to 1.0 with 1.0 being
     *                     equivilent to 100%, allowed to be considered at velocity.
     * @return true if the motors current velocity is within the given percent
     *         error, false otherwise.
     */
    public boolean isLowerShooterAtVelocity(double percentError) {
        percentError = Math.abs(percentError);
        percentError = percentError > 1 ? 1.0 : percentError;
        final double acceptableError = Math.abs(desiredLowerVelocity) * percentError;
        return Math.abs(desiredLowerVelocity - lowerVelocity) < acceptableError;
    }

    /**
     * Returns the velocity of the upper shooter motor in RPM, converted from Units
     * per 100ms.
     * 
     * @return The RPM of the upper shooter motor.
     */
    public double getUpperShooterVelocity() {
        return upperVelocity;
    }

    /**
     * Returns if the velocity of the motor is within the provided percent error
     * margin.
     * 
     * @param percentError The percent error, from 0.0 to 1.0 with 1.0 being
     *                     equivilent to 100%, allowed to be considered at velocity.
     * @return true if the motors current velocity is within the given percent
     *         error, false otherwise.
     */
    public boolean isUpperShooterAtVelocity(double percentError) {
        percentError = Math.abs(percentError);
        percentError = percentError > 1 ? 1.0 : percentError;
        final double acceptableError = Math.abs(desiredUpperVelocity) * percentError;
        return Math.abs(desiredUpperVelocity - upperVelocity) < acceptableError;
    }

    /**
     * Returns the target velocity of the lower shooter motor in RPM, converted from
     * Units per 100ms.
     * 
     * @return The target RPM of the lower shooter motor.
     */
    public double getLowerShooterTargetVelocity() {
        return desiredLowerVelocity;
    }

    /**
     * Returns the target velocity of the upper shooter motor in RPM, converted from
     * Units per 100ms.
     * 
     * @return The RPM of the upper shooter motor.
     */
    public double getUpperShooterTargetVelocity() {
        return desiredUpperVelocity;
    }

    /**
     * Returns if the velocity of the both shooter motors is within the
     * provided percent error margin.
     * 
     * @param percentError The percent error, from 0.0 to 1.0 with 1.0 being
     *                     equivilent to 100%, allowed to be considered at velocity.
     * @return true if the motors current velocity is within the given percent
     *         error, false otherwise.
     */
    public boolean isBothShootersAtVelocity(double percentError) {
        percentError = Math.abs(percentError);
        percentError = percentError > 1 ? 1.0 : percentError;
        final double lowerAcceptableError = Math.abs(desiredLowerVelocity) * percentError;
        final double upperAcceptableError = Math.abs(desiredUpperVelocity) * percentError;
        return Math.abs(desiredLowerVelocity - lowerVelocity) < lowerAcceptableError
                && Math.abs(desiredUpperVelocity - upperVelocity) < upperAcceptableError;
    }

    /**
     * Sets the speed of the feeder motors to the given value.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the feeder motors to.
     */
    public void setLowerFeederSpeed(final double speed) {
        if (!lowerFeederExecutorActive.get()) {
            lowerFeederMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * Sets the speed of the feeder motors to the given value and keeps them running
     * for the desired time.
     * 
     * @param speed   The speed, from -1.0 to 1.0, to set the feeder motor to.
     * @param runTime The amount of time, in seconds, to keep the motors spinning
     *                for.
     */
    public void runLowerFeeder(final double speed, final double runTime) {
        try {
            executorLock.lock();
            if (lowerFeederExecutorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        final double endTime = Timer.getFPGATimestamp() + runTime;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.isDisabled()) {
                                break;
                            }
                            lowerFeederMotor.set(ControlMode.PercentOutput, speed);
                            Timer.delay(TimedRobot.kDefaultPeriod);
                        }
                        lowerFeederMotor.set(ControlMode.PercentOutput, 0);
                    } finally {
                        lowerFeederExecutorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    /**
     * Sets the speed of the feeder motors to the given value.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the feeder motors to.
     */
    public void setUpperFeederSpeed(final double speed) {
        if (!upperFeederExecutorActive.get()) {
            upperFeederMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * Sets the speed of the feeder motors to the given value and keeps them running
     * for the desired time.
     * 
     * @param speed   The speed, from -1.0 to 1.0, to set the feeder motor to.
     * @param runTime The amount of time, in seconds, to keep the motors spinning
     *                for.
     */
    public void runUpperFeeder(final double speed, final double runTime) {
        try {
            executorLock.lock();
            if (upperFeederExecutorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        final double endTime = Timer.getFPGATimestamp() + runTime;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.isDisabled()) {
                                break;
                            }
                            upperFeederMotor.set(ControlMode.PercentOutput, speed);
                            Timer.delay(TimedRobot.kDefaultPeriod);
                        }
                        upperFeederMotor.set(ControlMode.PercentOutput, 0);
                    } finally {
                        upperFeederExecutorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    /**
     * Sets the speed of the feeder motors to the given value and keeps them running
     * for the desired time.
     * 
     * @param speed   The speed, from -1.0 to 1.0, to set the feeder motor to.
     * @param runTime The amount of time, in seconds, to keep the motors spinning
     *                for.
     */
    public void pulseUpperFeeder(final double speed, final double runTime) {
        try {
            executorLock.lock();
            if (upperFeederExecutorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        double endTime = Timer.getFPGATimestamp() + runTime;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.isDisabled()) {
                                break;
                            }
                            upperFeederMotor.set(ControlMode.PercentOutput, speed);
                            Timer.delay(.02);
                        }
                        endTime = Timer.getFPGATimestamp() + runTime;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.isDisabled()) {
                                break;
                            }
                            upperFeederMotor.set(ControlMode.PercentOutput, 0);
                            Timer.delay(.02);
                        }
                        upperFeederMotor.set(ControlMode.PercentOutput, 0);
                    } finally {
                        upperFeederExecutorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    /**
     * Sets the speed of the pickup motor to the given value.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the pickup motors to.
     */
    public void setPickupSpeed(final double speed) {
        pickupMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Returns the number of rotations that the pickup arm encoder differs from its
     * starting position.
     * 
     * @return The difference in the number of rotations that the motor is currently
     *         located at relative to its startup position.
     */
    public double getPickupArmPoisition() {
        return pickupArmPosition;
    }

    public double getPickupArmCurrent() {
        return pickupArmCurrent;
    }

    /**
     * Sets the speed of the pickup arm motor to the given value.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the pickup motors to.
     */
    public void setPickupArmSpeed(final double speed) {
        pickupArmMotor.set(speed);
    }

    public void extendPickupArm() {
        if (pickupArmPosition > 81) {
            pickupArmMotor.disable();
        } else {
            pickupArmPIDController.setReference(81, ControlType.kPosition);
        }
    }

    public void retractPickupArm() {
        if (pickupArmPosition < 10) {
            pickupArmMotor.disable();
        } else {
            pickupArmPIDController.setReference(10, ControlType.kPosition);
        }
    }

    /**
     * Disables all motors in the {@link Shooter} class.
     */
    public final void disable() {
        lowerShooterMotor.set(ControlMode.Disabled, 0);
        upperShooterMotor.set(ControlMode.Disabled, 0);
        pickupArmMotor.setIdleMode(IdleMode.kBrake);
        pickupMotor.set(ControlMode.Disabled, 0);
        lowerFeederMotor.set(ControlMode.Disabled, 0);
        upperFeederMotor.set(ControlMode.Disabled, 0);
        pickupArmMotor.disable();
    }

}