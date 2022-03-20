/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package frc.robot;

// Java Imports
import java.util.concurrent.ConcurrentLinkedQueue;
import java.io.IOException;

// FRC Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

// Team 3171 Imports
import frc.team3171.auton.AutonRecorder;
import frc.team3171.auton.AutonRecorderData;
import frc.team3171.auton.HardcodedAutons;
import frc.team3171.controllers.Climber;
import frc.team3171.controllers.Shooter;
//import frc.team3171.auton.HardcodedAutons;
import static frc.team3171.HelperFunctions.Deadzone_With_Map;
import frc.team3171.drive.UniversalMotorGroup;
import frc.team3171.drive.TractionDrive;
import frc.team3171.drive.UniversalMotorGroup.ControllerType;
import frc.team3171.sensors.GyroPIDController;
import frc.team3171.sensors.NavXMXP;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements RobotProperties {

  // Auton Recorder
  private AutonRecorder autonRecorder;
  private ConcurrentLinkedQueue<AutonRecorderData> autonPlaybackQueue;
  private AutonRecorderData playbackData;
  private double autonStartTime;
  private boolean saveNewAuton;

  // Auton Mode Constants
  private static final String kDefaultAuton = "Disabled";
  private static final String kHardcodedAuton = "Hardcoded Auton";

  // Selected Auton String
  private boolean selectedAutonType;
  private String selectedAutonMode;

  // Auton Chooser
  private SendableChooser<Boolean> autonTypeChooser;
  private SendableChooser<String> autonModeChooser;

  // Joysticks
  private Joystick leftStick, rightStick, operatorLeftStick, operatorRightStick;

  // Drive Controller
  private UniversalMotorGroup leftMotorGroup, rightMotorGroup;
  private TractionDrive driveController;
  private NavXMXP gyro;
  private GyroPIDController gyroPIDController;

  // Shooter Controller
  private Shooter shooterController;
  private DigitalInput feedSensor;
  private double shooterAtSpeedStartTime, maxCurrent;
  private boolean shooterButtonEdgeTrigger, shooterAtSpeedEdgeTrigger, ballpickupEdgeTrigger;

  // Climber Controller
  private Climber climberController;

  // Shooter PID Logging
  private ConcurrentLinkedQueue<String> outgoingMessages;
  private UDPClient udpClient = null;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    final double startTime = Timer.getFPGATimestamp();

    // Gyro init
    gyro = new NavXMXP();
    gyroPIDController = new GyroPIDController(gyro, GYRO_KP, GYRO_KI, GYRO_KD, -.75, .75);
    gyroPIDController.start();

    // Auton Recorder init
    autonRecorder = new AutonRecorder();
    autonPlaybackQueue = new ConcurrentLinkedQueue<>();
    playbackData = null;
    saveNewAuton = false;

    // Auton Type init
    selectedAutonType = false;
    autonTypeChooser = new SendableChooser<>();
    autonTypeChooser.setDefaultOption("Playback Auton", false);
    autonTypeChooser.addOption("Record Auton", true);
    SmartDashboard.putData("Auton Type:", autonTypeChooser);

    // Auton Modes init
    selectedAutonMode = kDefaultAuton;
    autonModeChooser = new SendableChooser<>();
    autonModeChooser.setDefaultOption(kDefaultAuton, kDefaultAuton);
    autonModeChooser.addOption(kHardcodedAuton, kHardcodedAuton);
    for (final String autonMode : AUTON_OPTIONS) {
      autonModeChooser.addOption(autonMode, autonMode);
    }
    SmartDashboard.putData("Auton Modes:", autonModeChooser);

    // Joystick init
    leftStick = new Joystick(0);
    rightStick = new Joystick(1);
    operatorLeftStick = new Joystick(2);
    operatorRightStick = new Joystick(3);

    // Drive, Shooter and Climber Controller inits
    try {
      leftMotorGroup = new UniversalMotorGroup(false, ControllerType.TalonFX, LEFT_DRIVE_CAN_ID_ARRAY);
      rightMotorGroup = new UniversalMotorGroup(false, ControllerType.TalonFX, RIGHT_DRIVE_CAN_ID_ARRAY);
      driveController = new TractionDrive(leftMotorGroup, rightMotorGroup);

      shooterController = new Shooter();
      climberController = new Climber();
    } catch (Exception e) {
      System.err.println(e.getMessage());
    }

    // Feed Sensor init
    feedSensor = new DigitalInput(FEED_SENSOR_CHANNEL);

    // Edge Trigger init
    shooterButtonEdgeTrigger = false;
    shooterAtSpeedEdgeTrigger = false;
    ballpickupEdgeTrigger = false;
    shooterAtSpeedStartTime = 0;
    maxCurrent = 0;

    // Camera Server for climber camera
    final UsbCamera camera0 = CameraServer.startAutomaticCapture();
    final UsbCamera camera1 = CameraServer.startAutomaticCapture();
    camera0.setResolution(160, 90);
    camera0.setFPS(20);
    camera1.setResolution(160, 90);
    camera1.setFPS(20);

    // PID Logging init
    if (PID_LOGGING && !DriverStation.isFMSAttached()) {
      try {
        outgoingMessages = new ConcurrentLinkedQueue<>();
        udpClient = new UDPClient(outgoingMessages, "10.31.71.201", 5801);
        udpClient.start();
      } catch (IOException e) {
        System.err.println("Invalid destination IP Address!");
      }
    }

    SmartDashboard.putString("roboInit:", String.format("%.4f", Timer.getFPGATimestamp() - startTime));
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    final double startTime = Timer.getFPGATimestamp();

    SmartDashboard.putString("Lower Shooter Velocity:", String.format("%d / %d",
        Math.round(shooterController.getLowerShooterVelocity()),
        Math.round(shooterController.getLowerShooterTargetVelocity())));
    SmartDashboard.putString("Upper Shooter Velocity:", String.format("%d / %d",
        Math.round(shooterController.getUpperShooterVelocity()),
        Math.round(shooterController.getUpperShooterTargetVelocity())));

    SmartDashboard.putBoolean("Feed Sensor:", feedSensor.get());
    SmartDashboard.putBoolean("NavX Present:", gyro.isConnected());
    SmartDashboard.putString("Gyro Lock:", String.format("%.2f", gyroPIDController.getSensorLockValue()));
    if (gyro.isConnected() && !gyro.isCalibrating()) {
      SmartDashboard.putString("NavX Heading:", String.format("%.2f", gyro.getYaw()));
    }

    // SmartDashboard.putNumber("Pickup Arm Position:",
    // shooterController.getPickupArmPoisition());
    SmartDashboard.putNumber("Primary Winch Position:", climberController.getPrimaryClimberPosition());
    SmartDashboard.putNumber("Secodary Winch One Position:", climberController.getSecondryClimberOnePosition());
    SmartDashboard.putNumber("Secondary Winch Two Position:", climberController.getSecondryClimberTwoPosition());

    if (PID_LOGGING && !DriverStation.isFMSAttached()) {
      outgoingMessages.add(String.format("%.4f,%.2f,%.2f,%.2f,0,0,0", Timer.getFPGATimestamp(),
          shooterController.getUpperShooterVelocity(), shooterController.getUpperShooterTargetVelocity(),
          shooterController.getUpperShooterSpeed()));
    }

    SmartDashboard.putNumber("Neo: Temp", shooterController.getPickupArmTemp());
    final double current = shooterController.getPickupArmCurrent();
    maxCurrent = current > maxCurrent ? current : maxCurrent;
    SmartDashboard.putNumber("Max Current", maxCurrent);

    SmartDashboard.putString("robotPeriodic:", String.format("%.4f", Timer.getFPGATimestamp() - startTime));
  }

  /**
   * This function is called once at the beginning of autonomous.
   */
  @Override
  public void autonomousInit() {
    final double startTime = Timer.getFPGATimestamp();

    // Reset all of the Edge Triggers
    shooterButtonEdgeTrigger = false;
    shooterAtSpeedEdgeTrigger = false;
    ballpickupEdgeTrigger = false;
    shooterAtSpeedStartTime = 0;

    // Reset Drive Direction
    driveController.setDriveDirectionFlipped(false);

    // Enable the Gyro PID Controller
    gyroPIDController.enablePID();

    // Update Auton Selected Mode and load the auton
    selectedAutonType = autonTypeChooser.getSelected();
    selectedAutonMode = autonModeChooser.getSelected();
    if (selectedAutonType) {
      playbackData = null;
    } else {
      switch (selectedAutonMode) {
        case kHardcodedAuton:
          HardcodedAutons.Auton_Init();
          break;
        case kDefaultAuton:
          disabledInit();
          playbackData = null;
          break;
        default:
          AutonRecorder.loadFromFile(autonPlaybackQueue, selectedAutonMode);
          playbackData = autonPlaybackQueue.poll();
          break;
      }
    }

    // Update the autonStartTime
    autonStartTime = Timer.getFPGATimestamp();

    SmartDashboard.putString("autonInit:", String.format("%.4f", Timer.getFPGATimestamp() - startTime));
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    final double startTime = Timer.getFPGATimestamp();

    switch (selectedAutonMode) {
      case kHardcodedAuton:
        HardcodedAutons.Auton_Basic(driveController, gyro, gyroPIDController, shooterController, feedSensor);
        break;
      case kDefaultAuton:
        disabledPeriodic();
        break;
      default:
        // Plays the recorded auton if theres a valid next step, otherwise disables
        if (playbackData != null) {
          // Get the latest joystick button values
          final boolean button_Pickup = playbackData.getPickup();
          final boolean button_Shooter = playbackData.getShooter();

          // Get the latest joystick values and calculate their deadzones
          final double leftStickY = playbackData.getLeftY(), rightStickX = playbackData.getRightX();

          // Drive Control
          if (gyroPIDController.isEnabled() && gyro.isConnected()) {
            if (rightStickX != 0) {
              driveController.mecanumTraction(-leftStickY, rightStickX);
              gyroPIDController.updateSensorLockValue();
            } else {
              driveController.mecanumTraction(-leftStickY, gyroPIDController.getPIDValue());
            }
          } else {
            driveController.mecanumTraction(-leftStickY, rightStickX);
          }

          // Shooter Control
          boolean extend_Pickup_Arm = false;
          if (button_Shooter && !shooterButtonEdgeTrigger) {
            // Sets the shooter speed and the targeting light
            shooterAtSpeedEdgeTrigger = false;
            shooterController.enableTargetingLight(true);
            shooterController.setShooterVelocity(LOWER_SHOOTER_VELOCITY, UPPER_SHOOTER_VELOCITY);
          } else if (button_Shooter) {
            // Check if the shooter is at speed
            final boolean isAtSpeed = shooterController.isBothShootersAtVelocity(DESIRED_PERCENT_ACCURACY);
            if (isAtSpeed && !shooterAtSpeedEdgeTrigger) {
              // Get time that shooter first designated at speed
              shooterAtSpeedStartTime = Timer.getFPGATimestamp();
            } else if (isAtSpeed && (Timer.getFPGATimestamp() >= shooterAtSpeedStartTime + DESIRED_AT_SPEED_TIME)) {
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
          // shooterController.setPickupArmSpeed(rightStick.getY());

          // Checks for new data and when to switch to it
          if ((Timer.getFPGATimestamp() - autonStartTime) >= playbackData.getFPGATimestamp()) {
            playbackData = autonPlaybackQueue.poll();
          }
        } else {
          disabledInit();
          selectedAutonMode = kDefaultAuton;
        }
        break;
    }

    SmartDashboard.putString("autonPeriodic:", String.format("%.4f", Timer.getFPGATimestamp() - startTime));
  }

  /**
   * This function is called once at the beginning of operator control.
   */
  @Override
  public void teleopInit() {
    final double startTime = Timer.getFPGATimestamp();

    // Reset all of the Edge Triggers
    shooterButtonEdgeTrigger = false;
    shooterAtSpeedEdgeTrigger = false;
    ballpickupEdgeTrigger = false;
    shooterAtSpeedStartTime = 0;

    // Reset Drive Direction
    driveController.setDriveDirectionFlipped(false);

    // Enable the Gyro PID Controller
    gyroPIDController.enablePID();

    // Update Auton Selected Mode and reset the data recorder
    selectedAutonType = autonTypeChooser.getSelected();
    selectedAutonMode = autonModeChooser.getSelected();
    autonRecorder.clear();
    saveNewAuton = selectedAutonType;

    // Update the autonStartTime
    autonStartTime = Timer.getFPGATimestamp();

    SmartDashboard.putString("teleopInit:", String.format("%.4f", Timer.getFPGATimestamp() - startTime));
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    final double startTime = Timer.getFPGATimestamp();

    // Get the latest joystick button values
    final boolean button_Pickup = leftStick.getTrigger();
    final boolean button_Boost = leftStick.getRawButton(2);
    final boolean button_Reverse_Pickup = leftStick.getRawButton(4);

    final boolean button_Shooter = rightStick.getTrigger();
    final boolean button_Short_Shot = rightStick.getRawButton(2);
    final boolean button_Full_Yeet = rightStick.getRawButton(3);
    final boolean button_Extend_Pickup_Arm = rightStick.getRawButton(4);

    final boolean button_Override_Primary_Climber = operatorLeftStick.getRawButton(2);
    final boolean button_Retract_Primary_Climber = operatorLeftStick.getRawButton(3);
    final boolean button_Extend_Primary_Climber = operatorLeftStick.getRawButton(4);

    final boolean button_Override_Secondary_Climber = operatorRightStick.getRawButton(2);
    final boolean button_Retract_Secondary_Climber = operatorRightStick.getRawButton(3);
    final boolean button_Extend_Secondary_Climber = operatorRightStick.getRawButton(4);

    // Get the latest joystick values and calculate their deadzones
    final double leftStickY, rightStickX, operatorLeftStickY, operatorRightStickY;
    if (button_Boost) {
      leftStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, leftStick.getY());
      rightStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, rightStick.getX());
    } else {
      leftStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, leftStick.getY()) * MAX_DRIVE_SPEED;
      rightStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, rightStick.getX()) * MAX_DRIVE_SPEED;
    }
    operatorLeftStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, operatorLeftStick.getY());
    operatorRightStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, operatorRightStick.getY()) * MAX_SECONDARY_CLIMBER_SPEED;

    // Drive Control
    if (gyroPIDController.isEnabled() && gyro.isConnected()) {
      if (rightStickX != 0) {
        driveController.mecanumTraction(-leftStickY, rightStickX);
        gyroPIDController.updateSensorLockValue();
      } else {
        driveController.mecanumTraction(-leftStickY, gyroPIDController.getPIDValue());
      }
    } else {
      driveController.mecanumTraction(-leftStickY, rightStickX);
    }

    // Disables the gyro if the climber is enaged at all
    if (button_Extend_Primary_Climber || button_Extend_Secondary_Climber || operatorLeftStickY != 0
        || operatorRightStickY != 0) {
      gyroPIDController.disablePID();
    }

    // Shooter Control
    boolean extend_Pickup_Arm = button_Extend_Pickup_Arm;
    if (button_Shooter && !shooterButtonEdgeTrigger) {
      // Sets the shooter speed and the targeting light
      shooterAtSpeedEdgeTrigger = false;
      shooterController.enableTargetingLight(true);
      shooterController.setShooterVelocity(LOWER_SHOOTER_VELOCITY, UPPER_SHOOTER_VELOCITY);
    } else if (button_Short_Shot && !shooterButtonEdgeTrigger) {
      // Sets the shooter speed for a short shot and the targeting light
      shooterAtSpeedEdgeTrigger = false;
      shooterController.enableTargetingLight(true);
      shooterController.setShooterVelocity(LOWER_SHOOTER_SHORT_VELOCITY, UPPER_SHOOTER_SHORT_VELOCITY);
    } else if (button_Shooter || button_Short_Shot) {
      // Check if the shooter is at speed
      final boolean isAtSpeed = shooterController.isBothShootersAtVelocity(DESIRED_PERCENT_ACCURACY);
      if (isAtSpeed && !shooterAtSpeedEdgeTrigger) {
        // Get time that shooter first designated at speed
        shooterAtSpeedStartTime = Timer.getFPGATimestamp();
      } else if (isAtSpeed && (Timer.getFPGATimestamp() >= shooterAtSpeedStartTime + DESIRED_AT_SPEED_TIME)) {
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
    } else if (button_Full_Yeet) {
      // Yeets the ball
      shooterAtSpeedEdgeTrigger = false;
      shooterController.setShooterSpeed(1);
      shooterController.setPickupSpeed(0);
      shooterController.setLowerFeederSpeed(SHOOTER_LOWER_FEED_SPEED);
      shooterController.setUpperFeederSpeed(SHOOTER_UPPER_FEED_SPEED);
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
      } else if (button_Reverse_Pickup) {
        shooterController.setPickupSpeed(REVERSE_PICKUP_SPEED);
        shooterController.setLowerFeederSpeed(REVERSE_LOWER_FEEDER_SPEED);
        shooterController.setUpperFeederSpeed(REVERSE_UPPER_FEEDER_SPEED);
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
    shooterButtonEdgeTrigger = button_Shooter || button_Short_Shot;
    ballpickupEdgeTrigger = button_Pickup;

    // Pickup Arm Control
    if (shooterController.getPickupArmCurrent() > PICKUP_ARM_MAX_CURRENT) {
      shooterController.setPickupArmSpeed(0);
    } else if (extend_Pickup_Arm) {
      shooterController.extendPickupArm();
    } else {
      shooterController.retractPickupArm();
    }
    // shooterController.setPickupArmSpeed(rightStick.getY());

    // Primary Climber Control
    if (button_Extend_Primary_Climber) {
      climberController.setPrimaryClimberSpeed(PRIMARY_CLIMBER_EXTEND_SPEED, PRIMARY_CLIMBER_MIN_TICK,
          PRIMARY_CLIMBER_MAX_TICK);
    } else if (button_Retract_Primary_Climber) {
      climberController.setPrimaryClimberSpeed(PRIMARY_CLIMBER_RETRACT_SPEED, PRIMARY_CLIMBER_MIN_TICK,
          PRIMARY_CLIMBER_MAX_TICK);
    } else if (button_Override_Primary_Climber) {
      climberController.setPrimaryClimberSpeed(-operatorLeftStickY);
    } else {
      climberController.setPrimaryClimberSpeed(-operatorLeftStickY, PRIMARY_CLIMBER_MIN_TICK, PRIMARY_CLIMBER_MAX_TICK);
    }

    // Secondary Climber Control
    if (button_Extend_Secondary_Climber) {
      climberController.setSecondaryClimberSpeed(SECONDARY_CLIMBER_EXTEND_SPEED, SECONDARY_CLIMBER_MIN_TICK,
          SECONDARY_CLIMBER_MAX_TICK);
    } else if (button_Retract_Secondary_Climber) {
      climberController.setSecondaryClimberSpeed(SECONDARY_CLIMBER_RETRACT_SPEED, SECONDARY_CLIMBER_MIN_TICK,
          SECONDARY_CLIMBER_MAX_TICK);
    } else if (button_Override_Secondary_Climber) {
      climberController.setSecondaryClimberSpeed(-operatorRightStickY);
    } else {
      climberController.setSecondaryClimberSpeed(-operatorRightStickY, SECONDARY_CLIMBER_MIN_TICK,
          SECONDARY_CLIMBER_MAX_TICK);
    }

    // Auton Recording
    if (saveNewAuton) {
      switch (selectedAutonMode) {
        case kHardcodedAuton:
        case kDefaultAuton:
          // Do Nothing
          break;
        default:
          final double autonFPGATimestamp = Timer.getFPGATimestamp() - autonStartTime;

          // Create new auton data packet and record the data
          final AutonRecorderData newData = new AutonRecorderData();
          newData.setFPGATimestamp(autonFPGATimestamp);
          newData.setLeftY(leftStickY);
          newData.setRightX(rightStickX);
          newData.setPickup(button_Pickup);
          newData.setShooter(button_Shooter);

          // Adds the recorded data to the auton recorder, but only if the data is new
          autonRecorder.addNewData(newData);
          break;
      }
    }

    SmartDashboard.putString("teleopPeriodic:", String.format("%.4f", Timer.getFPGATimestamp() - startTime));
  }

  /**
   * This function is called once at the beginning of the disabled mode.
   */
  @Override
  public void disabledInit() {
    final double startTime = Timer.getFPGATimestamp();

    // Disabled all of the robot controllers
    driveController.disable();
    shooterController.disable();
    climberController.disable();

    gyroPIDController.disablePID();

    if (saveNewAuton) {
      saveNewAuton = false;
      // Once auton recording is done, save the data to a file, if there is any
      switch (selectedAutonMode) {
        case kHardcodedAuton:
        case kDefaultAuton:
          // Do Nothing
          break;
        default:
          autonRecorder.saveToFile(selectedAutonMode);
          break;
      }
    }

    SmartDashboard.putString("disabledInit:", String.format("%.4f", Timer.getFPGATimestamp() - startTime));
  }

  /**
   * This function is called periodically during disabled mode.
   */
  @Override
  public void disabledPeriodic() {
    final double startTime = Timer.getFPGATimestamp();

    SmartDashboard.putString("disabledPeriodic:", String.format("%.4f", Timer.getFPGATimestamp() - startTime));
  }

}
