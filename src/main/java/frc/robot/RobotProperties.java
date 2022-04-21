package frc.robot;

/**
 * @author Mark Ebert
 */
public interface RobotProperties {

        /** Drive Variables **/
        public static final double JOYSTICK_DEADZONE = .08;
        public static final double MAX_DRIVE_SPEED = .75;

        public static boolean SHOW_SHOOTER_LOCK_DEBUG = false, SHOW_WINCH_TICKS = false, SHOW_LIMELIGHT_DEBUG = false;

        /** Shooter Variables **/
        public static class LowShot {
                public static final int LOWER_VELOCITY = 500, UPPER_VELOCITY = 1925;
                public static final double DESIRED_PERCENT_ACCURACY = .06, DESIRED_AT_SPEED_TIME = .3;
        }

        static class MidShot {
                public static final int LOWER_VELOCITY = 2700, UPPER_VELOCITY = 2400;
                public static final double DESIRED_PERCENT_ACCURACY = .05, DESIRED_AT_SPEED_TIME = .2;
        }

        static class HighShot {
                public static final int LOWER_VELOCITY = 5200, UPPER_VELOCITY = 200;
                public static final double DESIRED_PERCENT_ACCURACY = .03, DESIRED_AT_SPEED_TIME = .4;
        }

        static class YEETShot {
                public static final int LOWER_VELOCITY = 1288, UPPER_VELOCITY = 5150;
                public static final double DESIRED_PERCENT_ACCURACY = .05, DESIRED_AT_SPEED_TIME = .3;
        }

        /** Pickup Variables **/
        public static final int PICKUP_ARM_MAX_CURRENT = 90;
        public static final double REVERSE_PICKUP_SPEED = -.5;
        public static final double PICKUP_SPEED = .55;

        /** Feeder Variables **/
        public static final double SHOOTER_LOWER_FEED_SPEED = .15, SHOOTER_UPPER_FEED_SPEED = .25;
        public static final double REVERSE_LOWER_FEEDER_SPEED = -.75, REVERSE_UPPER_FEEDER_SPEED = -.75;
        public static final double LOWER_FEEDER_SPEED = .3, LOWER_FEEDER_SPEED_SLOW = .2;
        public static final double UPPER_FEEDER_SPEED = .2, UPPER_FEEDER_BACKFEED_SPEED = -.4;
        public static final double LOWER_FEED_END_SPEED = .15, LOWER_FEED_END_TIME = .2;
        public static final double UPPER_FEED_END_SPEED = -.3, UPPER_FEED_END_TIME = .2;

        /** Climber Variables **/
        public static final double MAX_SECONDARY_CLIMBER_SPEED = .4;
        public static final double PRIMARY_CLIMBER_RETRACT_SPEED = -1, PRIMARY_CLIMBER_EXTEND_SPEED = 1;
        public static final double SECONDARY_CLIMBER_RETRACT_SPEED = -.5, SECONDARY_CLIMBER_EXTEND_SPEED = .25;
        public static final int PRIMARY_CLIMBER_MIN_TICK = -460000, PRIMARY_CLIMBER_MAX_TICK = 935000;
        public static final int SECONDARY_CLIMBER_MIN_TICK = 5000, SECONDARY_CLIMBER_MAX_TICK = 165500;

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
        public static final double GYRO_KP = .0045, GYRO_KI = .0001, GYRO_KD = .0001;
        public static final double LIMELIGHT_KP = .02, LIMELIGHT_KI = .003, LIMELIGHT_KD = .00175;
        public static final double SHOOTER_KP = .01, SHOOTER_KI = .0002, SHOOTER_KD = .0001, SHOOTER_KF = 0;
        public static final double WINCH_KP = .01, WINCH_KI = .0002, WINCH_KD = .0001, WINCH_KF = 0;
        public static final double DRIVE_KP = .01, DRIVE_KI = .0002, DRIVE_KD = .0001, DRIVE_KF = 0;
        public static final double PICKUP_ARM_KP = 1.75e-2, PICKUP_ARM_KI = 2e-6, PICKUP_ARM_KD = 3e-6,
                        PICKUP_ARM_KF = 0.00412;

        /** Auton Properties **/
        public static final String[] AUTON_OPTIONS = {
                        "Position 1 - 2 Ball", "Position 1 - 3 Ball",
                        "Position 2 - 2 Ball", "Position 2 - 3 Ball",
                        "Position 3 - 2 Ball", "Position 3 - 3 Ball",
        };

}