package frc.robot.subsystems.helper;

import com.revrobotics.RelativeEncoder;

import frc.robot.Interfaces.Motors.MotorWithEncoder;

public class SparkMaxBrushlessEncoderMotor extends SparkMaxBrushlessMotor implements MotorWithEncoder{
    private RelativeEncoder encoderMotor;

    public SparkMaxBrushlessEncoderMotor(int canBusId, String motorName, RelativeEncoder encoder)
    {
        super(canBusId, motorName);
        this.encoderMotor = encoder;
    }

    @Override
    public double getPosition() {
        return this.encoderMotor.getPosition();
    }

    @Override
    public double getAngle() {
        return this.encoderMotor.getPosition();
    }

    @Override
    public double getRadian() {
        return (this.getAngle() * Math.PI) / 180;
    }
    
}
