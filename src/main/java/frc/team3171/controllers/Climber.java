package frc.team3171.controllers;

import frc.robot.RobotProperties;
import frc.team3171.drive.FRCTalonFX;

/**
 * @author Mark Ebert
 */
public class Climber implements RobotProperties {

    private final FRCTalonFX winchMotor;

    public Climber() throws Exception {
        winchMotor = new FRCTalonFX(WINCH_CAN_ID);
        winchMotor.setInverted(WINCH_INVERTED);
    }

    public void setClimberSpeed(final double speed) {
        winchMotor.set(speed);
    }

    public void disable() {
        winchMotor.disable();
    }

}
