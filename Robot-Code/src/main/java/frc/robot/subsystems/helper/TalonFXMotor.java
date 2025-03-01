package frc.robot.subsystems.helper;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interfaces.Motors.Motor;

public class TalonFXMotor implements Motor{

    private TalonFX motor;
    private final String TYPE = "TalonFX";
    private int id;
    private String name;

    public double limit = .1;

    public TalonFXMotor(int canBusId, String motorName)
    {
        this.name = motorName;
        this.id = canBusId;

        this.motor = new TalonFX(id);
    }

    @Override
    public void setSpeed(double speed) {
        double clampedValue = MathUtil.clamp(speed, -this.limit, this.limit);
        this.motor.set(clampedValue);
    }

    @Override
    public double getSpeed() {
        return this.motor.get();
    }

    @Override
    public String getName() {
        return String.format("%s (%s %d)", this.name, this.TYPE, this.id);
    }

    @Override
    public void printToSmartDashboard() {
        double position = this.motor.getPosition().getValueAsDouble();
        double angle = 20.7684319834 * position + 12.1284319834;
        SmartDashboard.putNumber(this.getName(), angle);
    }

    @Override
    public void enableBrake() {
        this.motor.setNeutralMode(NeutralModeValue.Brake);
    }
    
}
