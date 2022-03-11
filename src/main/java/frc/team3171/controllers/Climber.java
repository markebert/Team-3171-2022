package frc.team3171.controllers;

// CTRE Imports
import com.ctre.phoenix.motorcontrol.ControlMode;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.drive.FRCTalonFX;

/**
 * @author Mark Ebert
 */
public class Climber implements RobotProperties {

    // Winch Motors
    private final FRCTalonFX primaryWinch, secondaryWinchOne, secondaryWinchTwo;

    /**
     * 
     */
    public Climber() {
        primaryWinch = new FRCTalonFX(PRIMARY_WINCH_CAN_ID);
        secondaryWinchOne = new FRCTalonFX(SECONDARY_WINCH_ONE_CAN_ID);
        secondaryWinchTwo = new FRCTalonFX(SECONDARY_WINCH_TWO_CAN_ID);

        primaryWinch.setInverted(PRIMARY_WINCH_INVERTED);
        secondaryWinchOne.setInverted(SECONDARY_WINCH_ONE_INVERTED);
        secondaryWinchTwo.setInverted(SECONDARY_WINCH_TWO_INVERTED);

        // Config the Velocity closed loop values in slot0
        primaryWinch.config_kP(0, WINCH_KP);
        primaryWinch.config_kI(0, WINCH_KI);
        primaryWinch.config_kD(0, WINCH_KD);
        primaryWinch.config_kF(0, WINCH_KF);

        secondaryWinchOne.config_kP(0, WINCH_KP);
        secondaryWinchOne.config_kI(0, WINCH_KI);
        secondaryWinchOne.config_kD(0, WINCH_KD);
        secondaryWinchOne.config_kF(0, WINCH_KF);

        secondaryWinchTwo.config_kP(0, WINCH_KP);
        secondaryWinchTwo.config_kI(0, WINCH_KI);
        secondaryWinchTwo.config_kD(0, WINCH_KD);
        secondaryWinchTwo.config_kF(0, WINCH_KF);
    }

    /**
     * 
     * @param speed
     */
    public void setPrimaryClimberSpeed(final double speed) {
        primaryWinch.set(speed);
    }

    /**
     * 
     * @param speed
     */
    public void setPrimaryClimberSpeed(final double speed, final int lowerBound, final int upperBound) {
        if (speed < 0 && primaryWinch.getEncoderValue() < lowerBound) {
            primaryWinch.set(ControlMode.PercentOutput, 0);
        } else if (speed > 0 && primaryWinch.getEncoderValue() > upperBound) {
            primaryWinch.set(ControlMode.PercentOutput, 0);
        } else {
            primaryWinch.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * 
     * @param position
     */
    public void setPrimaryClimberPosition(final int position) {
        primaryWinch.set(ControlMode.Position, position);
    }

    public int getPrimaryClimberPosition() {
        return primaryWinch.getEncoderValue();
    }

    /**
     * 
     * @param speed
     */
    public void setSecondaryClimberSpeed(final double speed) {
        secondaryWinchOne.set(ControlMode.PercentOutput, speed);
        secondaryWinchTwo.set(ControlMode.PercentOutput, speed);
    }

    /**
     * 
     * @param speed
     */
    public void setSecondaryClimberSpeed(final double speed, final int lowerBound, final int upperBound) {
        if (speed < 0 && secondaryWinchOne.getEncoderValue() < lowerBound) {
            secondaryWinchOne.set(ControlMode.PercentOutput, 0);
        } else if (speed > 0 && secondaryWinchOne.getEncoderValue() > upperBound) {
            secondaryWinchOne.set(ControlMode.PercentOutput, 0);
        } else {
            secondaryWinchOne.set(ControlMode.PercentOutput, speed);
        }

        if (speed < 0 && secondaryWinchTwo.getEncoderValue() < lowerBound) {
            secondaryWinchTwo.set(ControlMode.PercentOutput, 0);
        } else if (speed > 0 && secondaryWinchTwo.getEncoderValue() > upperBound) {
            secondaryWinchTwo.set(ControlMode.PercentOutput, 0);
        } else {
            secondaryWinchTwo.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * 
     * @param position
     */
    public void setSecondaryClimberPosition(final int position) {
        if (position < 0) {
            secondaryWinchOne.set(ControlMode.PercentOutput, 0);
            secondaryWinchTwo.set(ControlMode.PercentOutput, 0);
        } else {
            secondaryWinchOne.set(ControlMode.Position, position);
            secondaryWinchTwo.set(ControlMode.Position, position);
        }
    }

    public int getSecondryClimberOnePosition() {
        return secondaryWinchOne.getEncoderValue();
    }

    public int getSecondryClimberTwoPosition() {
        return secondaryWinchTwo.getEncoderValue();
    }

    /**
     * 
     */
    public void disable() {
        primaryWinch.disable();
        secondaryWinchOne.disable();
        secondaryWinchTwo.disable();
    }

}
