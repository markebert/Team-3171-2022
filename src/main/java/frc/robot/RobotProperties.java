package frc.robot;

/**
 * @author Mark Ebert
 */
public interface RobotProperties {

        /**
         * Static Values Properties
         * 
         * @param JOYSTICK_DEADZONE            The percent of error allowed, from -1.0
         *                                     to
         *                                     1.0, of
         *                                     the joysticks X and Y values.
         * @param MAX_DRIVE_SPEED              The maximum drive speed of the robot,
         *                                     from
         *                                     0 to 1.0
         * @param MAX_SECONDARY_CLIMBER_SPEED
         * @param LOWER_SHOOTER_VELOCITY
         * @param UPPER_SHOOTER_VELOCITY
         * @param LOWER_SHOOTER_SHORT_VELOCITY
         * @param UPPER_SHOOTER_SHORT_VELOCITY
         * @param DESIRED_PERCENT_ACCURACY
         * @param DESIRED_AT_SPEED_TIME
         */
        public static final double JOYSTICK_DEADZONE = .08;
        public static final double MAX_DRIVE_SPEED = .75;
        public static final double MAX_SECONDARY_CLIMBER_SPEED = .4;
        public static final int LOWER_SHOOTER_VELOCITY = 1500, UPPER_SHOOTER_VELOCITY = 4000; // High shot
        public static final int LOWER_SHOOTER_SHORT_VELOCITY = 1250, UPPER_SHOOTER_SHORT_VELOCITY = 1900; // Low shot
        public static final double DESIRED_PERCENT_ACCURACY = .075, DESIRED_AT_SPEED_TIME = .1; // Accuracy Settings

        /**
         * CAN ID Properties
         * 
         * @param LEFT_DRIVE_CAN_ID_ARRAY    An int array representing the CAN IDs used
         *                                   for the left drive motors.
         * @param RIGHT_DRIVE_CAN_ID_ARRAY   An int array representing the CAN IDs used
         *                                   for the right drive motors.
         * @param PICKUP_MOTOR_CAN_ID        An int representing the CAN ID of the
         *                                   motor.
         * @param LOWER_FEEDER_CAN_ID        An int representing the CAN ID of the
         *                                   motor.
         * @param PRIMARY_WINCH_CAN_ID       An int representing the CAN ID of the
         *                                   motor.
         * @param LOWER_SHOOTER_CAN_ID       An int representing the CAN ID of the
         *                                   motor.
         * @param UPPER_SHOOTER_CAN_ID       An int representing the CAN ID of the
         *                                   motor.
         * @param UPPER_FEEDER_CAN_ID        An int representing the CAN ID of the
         *                                   motor.
         * @param SECONDARY_WINCH_ONE_CAN_ID An int representing the CAN ID of the
         *                                   motor.
         * @param SECONDARY_WINCH_TWO_CAN_ID An int representing the CAN ID of the
         *                                   motor.
         * @param PICKUP_ARM_CAN_ID          An int representing the CAN ID of the
         *                                   motor.
         */
        public static final int[] LEFT_DRIVE_CAN_ID_ARRAY = new int[] { 1, 2 },
                        RIGHT_DRIVE_CAN_ID_ARRAY = new int[] { 3, 4 };
        public static final int PICKUP_MOTOR_CAN_ID = 5;
        public static final int LOWER_FEEDER_CAN_ID = 6;
        public static final int PRIMARY_WINCH_CAN_ID = 7;
        public static final int LOWER_SHOOTER_CAN_ID = 8, UPPER_SHOOTER_CAN_ID = 9;
        public static final int UPPER_FEEDER_CAN_ID = 10;
        public static final int SECONDARY_WINCH_ONE_CAN_ID = 11, SECONDARY_WINCH_TWO_CAN_ID = 12;
        public static final int PICKUP_ARM_CAN_ID = 13;

        /**
         * Inversion Properties
         * 
         * @param LOWER_SHOOTER_INVERTED       Whether or not the lower shooter motor
         *                                     need to be inverted.
         * @param UPPER_SHOOTER_INVERTED       Whether or not the upper shooter motor
         *                                     need to be inverted.
         * @param PICKUP_MOTOR_INVERTED        Whether or not the pickup motor needs to
         *                                     be inverted.
         * @param PICKUP_MOTOR_INVERTED        Whether or not the pickup arm motor needs
         *                                     to be inverted.
         * @param LOWER_FEEDER_INVERTED        Whether or not the feeder motors needs to
         *                                     be inverted.
         * @param UPPER_FEEDER_INVERTED        Whether or not the feeder motor needs to
         *                                     be inverted.
         * @param PRIMARY_WINCH_INVERTED       Whether or not the winch motors needs to
         *                                     be inverted.
         * @param SECONDARY_WINCH_ONE_INVERTED Whether or not second winch one needs to
         *                                     be inverted.
         * @param SECONDARY_WINCH_TWO_INVERTED Whether or not second winch one needs
         *                                     to be inverted.
         */
        public static final boolean LOWER_SHOOTER_INVERTED = false, UPPER_SHOOTER_INVERTED = true;
        public static final boolean PICKUP_MOTOR_INVERTED = false, PICKUP_ARM_MOTOR_INVERTED = false;
        public static final boolean LOWER_FEEDER_INVERTED = false, UPPER_FEEDER_INVERTED = false;
        public static final boolean PRIMARY_WINCH_INVERTED = false;
        public static final boolean SECONDARY_WINCH_ONE_INVERTED = true, SECONDARY_WINCH_TWO_INVERTED = false;

        /**
         * Relay Channels. 0-4 are on-board.
         * 
         * @param TARGET_LIGHT_CHANNEL The relay channel to use for the targeting light
         *                             relay.
         */
        public static final int TARGET_LIGHT_CHANNEL = 0;

        /**
         * Digital Inputs and Outputs. 0-9 are on-board, 10-25 are on the MXP.
         * 
         * @param FEED_SENSOR_CHANNEL The dio channel to use for the feed sensor.
         */
        public static final int FEED_SENSOR_CHANNEL = 9;

        /**
         * PID Properties
         * 
         * @param PID_LOGGING   Whether or not to enable logging of the set PID
         *                      Controller.
         * @param GYRO_KP       The proportional value for the Gyro PID
         *                      Controller.
         * @param GYRO_KI       The integral value for the Gyro PID Controller.
         * @param GYRO_KD       The derivative value for the Gyro PID
         *                      Controller.
         * @param LIMELIGHT_KP  The proportional value for the Limelight PID
         *                      Controller.
         * @param LIMELIGHT_KI  The integral value for the Limelight PID
         *                      Controller.
         * @param LIMELIGHT_KD  The derivative value for the Limelight PID
         *                      Controller.
         * @param SHOOTER_KP    The proportional value for the Shooter PID
         *                      Controller.
         * @param SHOOTER_KI    The integral value for the Shooter PID
         *                      Controller.
         * @param SHOOTER_KD    The derivative value for the Shooter PID
         *                      Controller.
         * @param SHOOTER_KF    The feed-forward value for the Shooter PID
         *                      Controller. kF: 1.0 represents output value to
         *                      TalonFX at 100%, 6380/600 represents Velocity
         *                      units per 100 ms at 100% output.
         * @param WINCH_KP      The proportional value for the Winch PID
         *                      Controller.
         * @param WINCH_KI      The integral value for the Winch PID
         *                      Controller.
         * @param WINCH_KD      The derivative value for the Winch PID
         *                      Controller.
         * @param WINCH_KF      The feed-forward value for the Winch PID
         *                      Controller. kF: 1.0 represents output value to
         *                      TalonFX at 100%, 6380/600 represents Velocity
         *                      units per 100 ms at 100% output.
         * @param DRIVE_KP      The proportional value for the Drive PID
         *                      Controller.
         * @param DRIVE_KI      The integral value for the Drive PID
         *                      Controller.
         * @param DRIVE_KD      The derivative value for the Drive PID
         *                      Controller.
         * @param DRIVE_KF      The feed-forward value for the Drive PID
         *                      Controller. kF: 1.0 represents output value to
         *                      TalonFX at 100%, 6380/600 represents Velocity
         *                      units per 100 ms at 100% output.
         * @param PICKUP_ARM_KP The proportional value for the Pickup Arm PID
         *                      Controller.
         * @param PICKUP_ARM_KI The integral value for the Pickup Arm PID
         *                      Controller.
         * @param PICKUP_ARM_KD The derivative value for the Pickup Arm PID
         *                      Controller.
         * @param PICKUP_ARM_KF The feed-forward value for the Pickup Arm PID
         *                      Controller. kF: 1.0 represents output value to
         *                      Neo Spark Max Brushless at 100%.
         */
        public static final boolean PID_LOGGING = true;
        public static final double GYRO_KP = .0045, GYRO_KI = .0001, GYRO_KD = .0001;
        public static final double LIMELIGHT_KP = .025, LIMELIGHT_KI = .003, LIMELIGHT_KD = .00175;
        public static final double SHOOTER_KP = .01, SHOOTER_KI = .0002, SHOOTER_KD = .0001, SHOOTER_KF = 0;
        public static final double WINCH_KP = .01, WINCH_KI = .0002, WINCH_KD = .0001, WINCH_KF = 0;
        public static final double DRIVE_KP = .01, DRIVE_KI = .0002, DRIVE_KD = .0001, DRIVE_KF = 0;
        public static final double PICKUP_ARM_KP = 1.75e-2, PICKUP_ARM_KI = 2e-6, PICKUP_ARM_KD = 3e-6,
                        PICKUP_ARM_KF = 0.00412;

        /** Auton Properties **/
        public static final String[] AUTON_OPTIONS = {
                        "Auton 1",
                        "Auton 2"
        };

}