package frc.robot;

/**
 * @author Mark Ebert
 */
public interface RobotProperties {

        /**
         * Static Values Properties
         * 
         * @param JOYSTICK_DEADZONE The percent of error allowed, from -1.0 to 1.0, of
         *                          the joysticks X and Y values.
         * @param MAX_DRIVE_SPEED   The maximum drive speed of the robot, from 0 to 1.0
         */
        public static final double JOYSTICK_DEADZONE = .08;
        public static final double MAX_DRIVE_SPEED = .75;

        /**
         * CAN ID Properties
         * 
         * @param LEFT_DRIVE_CAN_ID_ARRAY   An int array representing the CAN IDs used
         *                                  for the first left drive motors.
         * @param RIGHT_DRIVE_CAN_ID_ARRAY  An int array representing the CAN IDs used
         *                                  for the first right drive motors.
         * @param LOWER_SHOOTER_CAN_ID      An int representing the CAN ID of the motor.
         * @param UPPER_SHOOTER_CAN_ID      An int representing the CAN ID of the motor.
         * @param PICKUP_MOTOR_CAN_ID       An int representing the CAN ID of the motor.
         * @param LOWER_FEEDER_CAN_ID_ARRAY An int array containing the CAN IDs of the
         *                                  motors.
         * @param UPPER_FEEDER_CAN_ID       An int representing the CAN ID of the motor.
         * @param WINCH_CAN_ID_ARRAY        An int array containing the CAN IDs of the
         *                                  motors.
         * @param PCM_CAN_ID                An int representing the CAN ID of the PCM.
         */
        public static final int[] LEFT_DRIVE_CAN_ID_ARRAY = new int[] { 1, 2 },
                        RIGHT_DRIVE_CAN_ID_ARRAY = new int[] { 3, 4 };
        public static final int LOWER_SHOOTER_CAN_ID = 8, UPPER_SHOOTER_CAN_ID = 9;
        public static final int PICKUP_MOTOR_CAN_ID = 5;
        public static final int LOWER_FEEDER_CAN_ID = 6;
        public static final int UPPER_FEEDER_CAN_ID = 10;
        public static final int WINCH_CAN_ID = 7;
        public static final int PCM_CAN_ID = 14;

        /**
         * Pneumatics Channels
         * 
         * @param PICKUP_ARM_FORWARD_CHANNEL    An int representing the pneumatic
         *                                      channel of the solenoids forward
         *                                      direction.
         * @param PICKUP_ARM_REVERSE_CHANNEL    An int representing the pneumatic
         *                                      channel of the solenoids reverse
         *                                      direction.
         * @param SHOOTER_BRAKE_FORWARD_CHANNEL An int representing the pneumatic
         *                                      channel of the solenoids forward
         *                                      direction.
         * @param SHOOTER_BRAKE_REVERSE_CHANNEL An int representing the pneumatic
         *                                      channel of the solenoids reverse
         *                                      direction.
         */
        public static final int PICKUP_ARM_FORWARD_CHANNEL = 0, PICKUP_ARM_REVERSE_CHANNEL = 1;
        public static final int SHOOTER_BRAKE_FORWARD_CHANNEL = 2, SHOOTER_BRAKE_REVERSE_CHANNEL = 3;

        /**
         * Inversion Properties
         * 
         * @param LOWER_SHOOTER_INVERTED Whether or not the lower shooter motor need to
         *                               be inverted.
         * @param UPPER_SHOOTER_INVERTED Whether or not the upper shooter motor need to
         *                               be inverted.
         * @param PICKUP_MOTOR_INVERTED  Whether or not the pickup motor needs to be
         *                               inverted.
         * @param LOWER_FEEDER_INVERTED  Whether or not the feeder motors needs to be
         *                               inverted.
         * @param UPPER_FEEDER_INVERTED  Whether or not the feeder motor needs to be
         *                               inverted.
         * @param WINCH_INVERTED         Whether or not the winch motors needs to be
         *                               inverted.
         * @param PICKUP_ARM_INVERTED    Whether or not the pickup arm piston needs to
         *                               be inverted.
         * @param SHOOTER_BRAKE_INVERTED Whether or not the shooter brake piston needs
         *                               to be inverted.
         */
        public static final boolean LOWER_SHOOTER_INVERTED = false, UPPER_SHOOTER_INVERTED = true;
        public static final boolean PICKUP_MOTOR_INVERTED = false;
        public static final boolean LOWER_FEEDER_INVERTED = false, UPPER_FEEDER_INVERTED = false;
        public static final boolean WINCH_INVERTED = false;
        public static final boolean PICKUP_ARM_INVERTED = true;
        public static final boolean SHOOTER_BRAKE_INVERTED = true;

        /**
         * Digital Inputs and Outputs. 0-9 are on-board, 10-25 are on the MXP.
         * 
         * @param TARGET_LIGHT_CHANNEL The channel to use for the targeting light relay.
         */
        public static final int TARGET_LIGHT_CHANNEL = 0;

        /**
         * PID Properties
         * 
         * @param GYRO_KP                The proportional value for the Gyro PID
         *                               Controller.
         * @param GYRO_KI                The integral value for the Gyro PID Controller.
         * @param GYRO_KD                The derivative value for the Gyro PID
         *                               Controller.
         * @param LIMELIGHT_KP           The proportional value for the Limelight PID
         *                               Controller.
         * @param LIMELIGHT_KI           The integral value for the Limelight PID
         *                               Controller.
         * @param LIMELIGHT_KD           The derivative value for the Limelight PID
         *                               Controller.
         * @param SHOOTER_KP             The proportional value for the Shooter PID
         *                               Controller.
         * @param SHOOTER_KI             The integral value for the Shooter PID
         *                               Controller.
         * @param SHOOTER_KD             The derivative value for the Shooter PID
         *                               Controller.
         * @param SHOOTER_KF             The feed-forward value for the Shooter PID
         *                               Controller. kF: 1.0 represents output value to
         *                               TalonFX at 100%, 6380/600 represents Velocity
         *                               units per 100 ms at 100% output.
         * @param SHOOTER_KPID_LOOPINDEX Sets whether or not the integrated PID
         *                               Controller of the Talon is closed loop (0) or
         *                               open loop (1).
         * @param SHOOTER_KTIMEOUT_MS    The timeout in ms to give the PID Controller of
         *                               the Talon.
         */
        public static final double GYRO_KP = .0045, GYRO_KI = .0001, GYRO_KD = .0001;
        public static final double LIMELIGHT_KP = .025, LIMELIGHT_KI = .003, LIMELIGHT_KD = .00175;
        public static final double SHOOTER_KP = .01, SHOOTER_KI = .0002, SHOOTER_KD = .0001, SHOOTER_KF = 0;
        public static final int SHOOTER_KPID_LOOPINDEX = 0, SHOOTER_KTIMEOUT_MS = 20;

        /** Auton Properties **/
        public static final String[] AUTON_OPTIONS = {
                        "Auton 1",
                        "Auton 2"
        };

}