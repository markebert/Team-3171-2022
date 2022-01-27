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
//import frc.team3171.auton.HardcodedAutons;
import static frc.team3171.HelperFunctions.Deadzone_With_Map;
import frc.team3171.drive.UniversalMotorGroup;
import frc.team3171.drive.TractionDrive;
import frc.team3171.drive.UniversalMotorGroup.MotorType;

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
  private static final String kRecordAutonOne = "Record Auton 1";
  private static final String kPlaybackAutonOne = "Playback Auton 1";

  // Selected Auton String
  private String selectedAutonMode;

  // Auton Chooser
  private SendableChooser<String> autonChooser;

  // Joysticks
  private Joystick leftStick, rightStick;

  // Drive Controller
  private UniversalMotorGroup leftMotorGroup, rightMotorGroup;
  private TractionDrive driveController;

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

    // Auton Modes init
    autonChooser = new SendableChooser<>();
    autonChooser.setDefaultOption(kDefaultAuton, kDefaultAuton);
    autonChooser.addOption(kHardcodedAuton, kHardcodedAuton);
    autonChooser.addOption(kRecordAutonOne, kRecordAutonOne);
    autonChooser.addOption(kPlaybackAutonOne, kPlaybackAutonOne);

    SmartDashboard.putData("Auton Modes:", autonChooser);

    // Joystick init
    leftStick = new Joystick(0);
    rightStick = new Joystick(1);

    // Drive, Shooter and Climber Controller inits
    try {
      leftMotorGroup = new UniversalMotorGroup(false, MotorType.TalonFX, leftDriveCANIDArray);
      rightMotorGroup = new UniversalMotorGroup(false, MotorType.TalonFX, rightDriveCANIDArray);
      driveController = new TractionDrive(leftMotorGroup, rightMotorGroup);
    } catch (Exception e) {
      System.err.println(e.getMessage());
    }

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
    selectedAutonMode = autonChooser.getSelected();
    switch (selectedAutonMode) {
      case kPlaybackAutonOne:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonOne);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kHardcodedAuton:
        //HardcodedAutons.Auton_Init();
      default:
        playbackData = null;
        break;
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
      case kPlaybackAutonOne:
        // Plays the recorded auton if theres a valid next step, otherwise disables
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
      case kHardcodedAuton:
        // HardcodedAutons.Auton_Center(driveController, gyro, gyroPIDController,
        // shooterController);
        break;
      case kRecordAutonOne:
      case kDefaultAuton:
      default:
        // Do Nothing
        break;
    }

    SmartDashboard.putNumber("autonomousPeriodic:", Timer.getFPGATimestamp() - startTime);
  }

  /**
   * This function is called once at the beginning of operator control.
   */
  @Override
  public void teleopInit() {
    final double startTime = Timer.getFPGATimestamp();

    // Reset Drive Direction
    driveController.setDriveDirectionFlipped(false);

    // Update Auton Selected Mode and reset the data recorder
    selectedAutonMode = autonChooser.getSelected();
    autonRecorder.clear();
    saveNewAuton = false;

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

    // Get the latest joystick values and calculate their deadzones
    final double leftStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, leftStick.getY());
    final double rightStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, rightStick.getX());

    // Auton Recording
    switch (selectedAutonMode) {
      case kRecordAutonOne:
        final double autonFPGATimestamp = Timer.getFPGATimestamp() - autonStartTime;
        saveNewAuton = true;
        // Crate new auton data packet and record the data
        final AutonRecorderData newData = new AutonRecorderData();
        newData.setFPGATimestamp(autonFPGATimestamp);

        newData.setLeftY(leftStickY);
        newData.setRightX(rightStickX);

        // Adds the recorded data to the auton recorder, but only if the data is new
        autonRecorder.addNewData(newData);
        break;
      default:
        // Do Nothing
        break;
    }

    driveController.mecanumTraction(-leftStickY, rightStickX);

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

    if (saveNewAuton) {
      saveNewAuton = false;
      // Once auton recording is done, save the data to a file, if there is any
      switch (selectedAutonMode) {
        case kRecordAutonOne:
          autonRecorder.saveToFile(kPlaybackAutonOne);
          break;
        default:
          // Do Nothing
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
