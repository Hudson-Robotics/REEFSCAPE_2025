package frc.robot.subsystems.helper;

import frc.robot.Interfaces.Motors.MotorWithEncoder;

public class TalonFXMotorWithEncoder extends TalonFXMotor implements MotorWithEncoder{

    public TalonFXMotorWithEncoder(int canBusId, String motorName)
    {
        super(canBusId, motorName);
    }

    @Override
    public double getPosition() {
        return this.motor.getPosition().getValueAsDouble();
    }

    @Override
    public double getAngle() {
        return this.getPosition(); // have an equation to convert this to an angle
    }

    @Override
    public double getRadian() {
        return (this.getAngle() * Math.PI) / 180;
    }

}
