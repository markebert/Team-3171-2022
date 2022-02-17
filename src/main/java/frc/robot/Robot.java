/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package frc.robot;

// Java Imports
import java.util.concurrent.ConcurrentLinkedQueue;

// FRC Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

// Team 3171 Imports
import frc.team3171.auton.AutonRecorder;
import frc.team3171.auton.AutonRecorderData;
import frc.team3171.controllers.Climber;
import frc.team3171.controllers.Shooter;

//import frc.team3171.auton.HardcodedAutons;
import static frc.team3171.HelperFunctions.Deadzone_With_Map;
import static frc.team3171.HelperFunctions.Within_Percent_Error;
import frc.team3171.drive.UniversalMotorGroup;
import frc.team3171.drive.TractionDrive;
import frc.team3171.drive.UniversalMotorGroup.ControllerType;

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
  private volatile double autonStartTime;
  private volatile boolean saveNewAuton;

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
  private Joystick leftStick, rightStick, operatorLeftStick;

  // Drive Controller
  private UniversalMotorGroup leftMotorGroup, rightMotorGroup;
  private TractionDrive driveController;

  // Shooter Controller
  private Shooter shooterController;
  private volatile boolean shooterAtSpeedEdgeTrigger, ballpickupEdgeTrigger, reverseFeederEdgeTrigger;

  // Climber Controller
  private Climber climberController;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    final double startTime = Timer.getFPGATimestamp();

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
    for (final String autonMode : autonOptions) {
      autonModeChooser.addOption(autonMode, autonMode);
    }
    SmartDashboard.putData("Auton Modes:", autonModeChooser);

    // Joystick init
    leftStick = new Joystick(0);
    rightStick = new Joystick(1);
    operatorLeftStick = new Joystick(2);

    // Drive, Shooter and Climber Controller inits
    try {
      leftMotorGroup = new UniversalMotorGroup(false, ControllerType.TalonSRX, leftDriveCANIDArray);
      rightMotorGroup = new UniversalMotorGroup(false, ControllerType.TalonSRX, rightDriveCANIDArray);
      driveController = new TractionDrive(leftMotorGroup, rightMotorGroup);

      shooterController = new Shooter();
      climberController = new Climber();
    } catch (Exception e) {
      System.err.println(e.getMessage());
    }

    // Edge Trigger init
    shooterAtSpeedEdgeTrigger = false;
    ballpickupEdgeTrigger = false;
    reverseFeederEdgeTrigger = false;

    SmartDashboard.putNumber("roboInit:", Timer.getFPGATimestamp() - startTime);
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

    SmartDashboard.putNumber("Lower Shooter Velocity:", shooterController.getLowerShooterVelocity());
    SmartDashboard.putNumber("Lower Shooter Target Velocity:", shooterController.getLowerShooterTargetVelocity());
    SmartDashboard.putNumber("Upper Shooter Velocity:", shooterController.getUpperShooterVelocity());
    SmartDashboard.putNumber("Upper Shooter Target Velocity:", shooterController.getUpperShooterTargetVelocity());

    SmartDashboard.putNumber("robotPeriodic:", Timer.getFPGATimestamp() - startTime);
  }

  /**
   * This function is called once at the beginning of autonomous.
   */
  @Override
  public void autonomousInit() {
    final double startTime = Timer.getFPGATimestamp();

    // Reset Drive Direction
    driveController.setDriveDirectionFlipped(false);

    // Update Auton Selected Mode and load the auton
    selectedAutonType = autonTypeChooser.getSelected();
    selectedAutonMode = autonModeChooser.getSelected();
    if (selectedAutonType) {
      playbackData = null;
    } else {
      switch (selectedAutonMode) {
        case kHardcodedAuton:
          // HardcodedAutons.Auton_Init();
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

    SmartDashboard.putNumber("autonomousInit:", Timer.getFPGATimestamp() - startTime);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    final double startTime = Timer.getFPGATimestamp();

    switch (selectedAutonMode) {
      case kHardcodedAuton:
        // HardcodedAutons.Auton_Center(driveController, gyro, gyroPIDController,
        // shooterController);
        break;
      case kDefaultAuton:
        disabledPeriodic();
        break;
      default: // Plays the recorded auton if theres a valid next step, otherwise disables
        if (playbackData != null) {
          // Get the latest joystick values and calculate their deadzones
          final double leftStickY = playbackData.getLeftY();
          final double rightStickX = playbackData.getRightX();

          driveController.mecanumTraction(-leftStickY, rightStickX);

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

    SmartDashboard.putNumber("autonomousPeriodic:", Timer.getFPGATimestamp() - startTime);
  }

  double shooterAtSpeedStart = 0;

  /**
   * This function is called once at the beginning of operator control.
   */
  @Override
  public void teleopInit() {
    final double startTime = Timer.getFPGATimestamp();

    shooterAtSpeedStart = startTime;

    // Reset all of the Edge Triggers
    shooterAtSpeedEdgeTrigger = false;
    ballpickupEdgeTrigger = false;
    reverseFeederEdgeTrigger = false;

    // Reset Drive Direction
    driveController.setDriveDirectionFlipped(false);

    // Update Auton Selected Mode and reset the data recorder
    selectedAutonType = autonTypeChooser.getSelected();
    selectedAutonMode = autonModeChooser.getSelected();
    autonRecorder.clear();
    saveNewAuton = selectedAutonType;

    // Update the autonStartTime
    autonStartTime = Timer.getFPGATimestamp();

    SmartDashboard.putNumber("teleopInit:", Timer.getFPGATimestamp() - startTime);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    final double startTime = Timer.getFPGATimestamp();

    // Get the latest joystick button values
    final boolean boost_Button = leftStick.getRawButton(2);
    final boolean button_Pickup = leftStick.getTrigger();
    final boolean button_Reverse_Pickup = leftStick.getRawButton(4);
    final boolean button_Shooter = rightStick.getTrigger();
    final boolean extendClimber = operatorLeftStick.getRawButton(4);
    final boolean retractClimber = operatorLeftStick.getRawButton(3);

    // Get the latest joystick values and calculate their deadzones
    final double leftStickY, rightStickX;
    if (boost_Button) {
      leftStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, leftStick.getY());
      rightStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, rightStick.getX());
    } else {
      leftStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, leftStick.getY()) * MAX_DRIVE_SPEED;
      rightStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, rightStick.getX()) * MAX_DRIVE_SPEED;
    }

    // Drive Control
    driveController.mecanumTraction(-leftStickY, rightStickX);

    // Shooter Control
    final int lowerShooterVelocity = 2500, upperShooterVelocity = 6000;
    if (button_Shooter) {
      shooterController.setShooterVelocity(lowerShooterVelocity, upperShooterVelocity);
      // shooterController.retractPickupArm();
      final boolean isAtSpeed = Within_Percent_Error(shooterController.getLowerShooterVelocity(), lowerShooterVelocity,
          .04)
          && Within_Percent_Error(shooterController.getUpperShooterVelocity(), upperShooterVelocity, .04);
      if (isAtSpeed && !shooterAtSpeedEdgeTrigger) {
        shooterAtSpeedStart = Timer.getFPGATimestamp();
      } else if (isAtSpeed && shooterAtSpeedEdgeTrigger && (Timer.getFPGATimestamp() >= shooterAtSpeedStart + 5)) {
        shooterController.setLowerFeederSpeed(.8);
        shooterController.setUpperFeederSpeed(.8);
      } else {
        shooterController.setLowerFeederSpeed(0);
        shooterController.setUpperFeederSpeed(0);
      }
      shooterAtSpeedEdgeTrigger = isAtSpeed;
    } else {
      shooterController.setShooterVelocity(0);
      // Ball Pickup Controls
      if (button_Pickup) {
        shooterController.extendPickupArm();
        // shooterController.setPickupSpeed(1);
        // final boolean pickupSensor = distanceSensor.getRange() < 125;
        final boolean pickupSensor = false;
        if (pickupSensor && !reverseFeederEdgeTrigger) {
          // shooterController.runLowerFeeder(-.2, .25);
          shooterController.runUpperFeeder(-.2, .25);
        } else {
          shooterController.setLowerFeederSpeed(.3);
          shooterController.setUpperFeederSpeed(.3);
        }
        reverseFeederEdgeTrigger = pickupSensor;
      } else if (button_Reverse_Pickup) {
        // shooterController.setPickupSpeed(0);
        shooterController.setLowerFeederSpeed(-.5);
        shooterController.setUpperFeederSpeed(-.5);
      } else {
        shooterController.setPickupSpeed(0);
        shooterController.retractPickupArm();
        shooterController.setLowerFeederSpeed(0);
        if (ballpickupEdgeTrigger) {
          shooterController.runUpperFeeder(-.2, .25);
        } else {
          shooterController.setUpperFeederSpeed(0);
        }
      }
      ballpickupEdgeTrigger = button_Pickup;
    }

    // Climber Control
    if (extendClimber) {
      climberController.setClimberSpeed(.5);
    } else if (retractClimber) {
      climberController.setClimberSpeed(-.5);
    } else {
      climberController.setClimberSpeed(0);
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

          // Adds the recorded data to the auton recorder, but only if the data is new
          autonRecorder.addNewData(newData);
          break;
      }
    }

    SmartDashboard.putNumber("teleopPeriodic:", Timer.getFPGATimestamp() - startTime);
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

    SmartDashboard.putNumber("disabledInit:", Timer.getFPGATimestamp() - startTime);
  }

  /**
   * This function is called periodically during disabled mode.
   */
  @Override
  public void disabledPeriodic() {
    final double startTime = Timer.getFPGATimestamp();

    SmartDashboard.putNumber("disabledPeriodic:", Timer.getFPGATimestamp() - startTime);
  }

}
