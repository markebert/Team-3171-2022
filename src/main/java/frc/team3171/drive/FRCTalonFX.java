package frc.team3171.drive;

// FRC Imports
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

// CTRE Imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;;

/**
 * Encapsulates the {@link TalonFX} to comply with the {@link MotorController}
 * interface so that it can function within FRC designed drive trains.
 * 
 * @author Mark Ebert
 */
public class FRCTalonFX extends TalonFX implements MotorController {

    /**
     * Constructor
     * 
     * @param deviceNumber [0,62]
     */
    public FRCTalonFX(int deviceNumber) {
        super(deviceNumber);
        configFactoryDefault();
        setNeutralMode(NeutralMode.Brake);
        configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        configNominalOutputForward(0);
        configNominalOutputReverse(0);
        configPeakOutputForward(1);
        configPeakOutputReverse(-1);
        setSelectedSensorPosition(0);
    }

    @Override
    public void set(double speed) {
        set(ControlMode.PercentOutput, speed);
    }

    @Override
    public double get() {
        return getMotorOutputPercent();
    }

    @Override
    public void disable() {
        set(ControlMode.Disabled, 0);
    }

    @Override
    public void stopMotor() {
        set(ControlMode.Disabled, 0);
    }

    @Override
    public void setInverted(final boolean invert) {
        super.setInverted(invert);
        setSensorPhase(invert);
    }

    /**
     * Returns the raw value of the {@link TalonFX} integrated encoder. The encoder
     * has 2048 ticks per revolution.
     * 
     * @return The raw value of the {@link TalonFX} integrated encoder.
     */
    public int getEncoderValue() {
        return (int) getSelectedSensorPosition();
    }

    /**
     * Returns the velocity of the {@link TalonFX} integrated encoder. The encoder
     * has 2048 ticks per revolution and the return units of the velocity is in
     * ticks per 100ms.
     * 
     * @return The velocity, in ticks per 100ms, of the {@link TalonFX} integrated
     *         encoder.
     */
    public int getEncoderVelocity() {
        return (int) getSelectedSensorVelocity();
    }

}