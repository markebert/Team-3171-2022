package frc.team3171.sensors;

// Java Importa
import java.util.function.DoubleSupplier;

// NavX MXP Imports
import com.kauailabs.navx.frc.AHRS;

// FRC Imports
import edu.wpi.first.wpilibj.SPI;

/**
 * @author Mark Ebert
 */
public class NavXMXP extends AHRS implements DoubleSupplier {

    /**
     * Instantiates the NavX MXP using the SPI MXP port.
     */
    public NavXMXP() {
        super(SPI.Port.kMXP);
    }

    /**
     * @return The yaw angle from the NavX MXP, which ranges from -180.0 - 180.0
     *         degrees.
     */
    @Override
    public double getAsDouble() {
        return getYaw();
    }

}
