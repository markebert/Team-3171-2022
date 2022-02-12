package frc.team3171.controllers;

import frc.robot.RobotProperties;
import frc.team3171.drive.UniversalMotorGroup;
import frc.team3171.drive.UniversalMotorGroup.ControllerType;

/**
 * @author Mark Ebert
 */
public class Climber implements RobotProperties {

    private final UniversalMotorGroup winchMotors;

    public Climber() throws Exception {
        winchMotors = new UniversalMotorGroup(winchInverted, ControllerType.TalonSRX, winchCANIDArray);
    }

    public void setClimberSpeed(final double speed) {
        winchMotors.set(speed);
    }

    public void disable() {
        winchMotors.disable();
    }

}