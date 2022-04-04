package frc.team3171.auton;

// FRC Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.controllers.Shooter;
import frc.team3171.drive.TractionDrive;
import frc.team3171.sensors.GyroPIDController;
import frc.team3171.sensors.NavXMXP;

/**
 * @author Mark Ebert
 */
public class HardcodedAutons implements RobotProperties {

    // Edge Triggers
    private static boolean shooterAtSpeedEdgeTrigger, shooterButtonEdgeTrigger, ballpickupEdgeTrigger;
    private static double shooterAtSpeedStartTime;

    // Auton Start Time
    private static double autonStartTime = 0;

    // Hidden Constructor
    private HardcodedAutons() {
    }

    /**
     * Must be called during {@link TimedRobot#autonomousInit()} prior to executing
     * any of the {@linkplain HardcodedAutons} to prevent any issues from occurring.
     */
    public static void Auton_Init() {
        shooterAtSpeedEdgeTrigger = false;
        shooterButtonEdgeTrigger = false;
        ballpickupEdgeTrigger = false;
        shooterAtSpeedStartTime = 0;
        autonStartTime = Timer.getFPGATimestamp();
    }

    /**
     * Auton suitable for use in the center starting position of the field.
     * 
     * @param driveController   The drive controller of the robot.
     * @param gyro              The gyro used by the robot.
     * @param gyroPIDController The pid controller used by the robot for driving.
     * @param shooterController The shooter controller of the robot.
     * @param feedSensor        The feed sensor used by the robot.
     */
    public static void Auton_Basic(final TractionDrive driveController, final NavXMXP gyro,
            final GyroPIDController gyroPIDController, final Shooter shooterController, final DigitalInput feedSensor) {
        final double currentTime = Timer.getFPGATimestamp() - autonStartTime;
        final double leftStickY, rightStickX;
        final boolean button_Shooter, button_Pickup;
        if (DriverStation.isAutonomousEnabled()) {
            // Set the controls at various time frames
            if (currentTime < 1.5) {
                leftStickY = -.2;
                rightStickX = 0;
                button_Shooter = false;
                button_Pickup = false;
            } else if (currentTime < 8) {
                leftStickY = 0;
                rightStickX = 0;
                button_Shooter = true;
                button_Pickup = false;
            } else if (currentTime < 10) {
                leftStickY = -.2;
                rightStickX = 0;
                button_Shooter = false;
                button_Pickup = false;
            } else {
                leftStickY = 0;
                rightStickX = 0;
                button_Shooter = false;
                button_Pickup = false;
            }

            // Drive Control
            if (gyroPIDController.isEnabled() && gyro.isConnected()) {
                if (rightStickX != 0) {
                    driveController.mecanumTraction(leftStickY, rightStickX);
                    gyroPIDController.updateSensorLockValue();
                } else {
                    driveController.mecanumTraction(leftStickY, gyroPIDController.getPIDValue());
                }
            } else {
                driveController.mecanumTraction(leftStickY, rightStickX);
            }

            // Shooter Control
            boolean extend_Pickup_Arm = false;
            if (button_Shooter && !shooterButtonEdgeTrigger) {
                // Sets the shooter speed and the targeting light
                shooterAtSpeedEdgeTrigger = false;
                shooterController.enableTargetingLight(true);
                shooterController.setShooterVelocity(MidShot.LOWER_VELOCITY, MidShot.UPPER_VELOCITY);
            } else if (button_Shooter) {
                // Check if the shooter is at speed
                final boolean isAtSpeed = shooterController.isBothShootersAtVelocity(MidShot.DESIRED_PERCENT_ACCURACY);
                if (isAtSpeed && !shooterAtSpeedEdgeTrigger) {
                    // Get time that shooter first designated at speed
                    shooterAtSpeedStartTime = Timer.getFPGATimestamp();
                } else if (isAtSpeed
                        && (Timer.getFPGATimestamp() >= shooterAtSpeedStartTime + MidShot.DESIRED_AT_SPEED_TIME)) {
                    // Feed the ball through the shooter
                    shooterController.setLowerFeederSpeed(SHOOTER_LOWER_FEED_SPEED);
                    shooterController.setUpperFeederSpeed(SHOOTER_UPPER_FEED_SPEED);
                } else if (!feedSensor.get()) {
                    // Back off the ball from the feed sensor
                    shooterController.setLowerFeederSpeed(0);
                    shooterController.setUpperFeederSpeed(UPPER_FEEDER_BACKFEED_SPEED);
                } else {
                    // Feeder stopped while shooter gets up tp speeed
                    shooterController.setLowerFeederSpeed(0);
                    shooterController.setUpperFeederSpeed(0);
                }
                shooterController.setPickupSpeed(0);
                shooterAtSpeedEdgeTrigger = isAtSpeed;
            } else {
                // Stops the shooter
                shooterAtSpeedEdgeTrigger = false;
                shooterController.enableTargetingLight(false);
                shooterController.setShooterSpeed(0);

                // Ball Pickup Controls
                if (button_Pickup) {
                    extend_Pickup_Arm = true;
                    shooterController.setPickupSpeed(PICKUP_SPEED);
                    if (!feedSensor.get()) {
                        shooterController.setLowerFeederSpeed(LOWER_FEEDER_SPEED_SLOW);
                        shooterController.setUpperFeederSpeed(0);
                    } else {
                        shooterController.setLowerFeederSpeed(LOWER_FEEDER_SPEED);
                        shooterController.setUpperFeederSpeed(UPPER_FEEDER_SPEED);
                    }
                } else if (ballpickupEdgeTrigger && !feedSensor.get()) {
                    shooterController.setPickupSpeed(0);
                    shooterController.runLowerFeeder(LOWER_FEED_END_SPEED, LOWER_FEED_END_TIME);
                    shooterController.runUpperFeeder(UPPER_FEED_END_SPEED, UPPER_FEED_END_TIME);
                    extend_Pickup_Arm = true;
                } else {
                    shooterController.setPickupSpeed(0);
                    shooterController.setLowerFeederSpeed(0);
                    shooterController.setUpperFeederSpeed(0);
                }
            }
            shooterButtonEdgeTrigger = button_Shooter;
            ballpickupEdgeTrigger = button_Pickup;

            // Pickup Arm Control
            if (shooterController.getPickupArmCurrent() > PICKUP_ARM_MAX_CURRENT) {
                shooterController.setPickupArmSpeed(0);
            } else if (extend_Pickup_Arm) {
                shooterController.extendPickupArm();
            } else {
                shooterController.retractPickupArm();
            }
        } else {
            driveController.disable();
            gyroPIDController.disablePID();
            shooterController.disable();
        }
    }

}