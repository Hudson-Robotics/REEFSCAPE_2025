package frc.robot.subsystems.helper;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interfaces.Motors.MotorWithEncoder;

public class SparkMaxBrushlessEncoderMotor extends SparkMaxBrushlessMotor implements MotorWithEncoder{
    private CANcoder encoderMotor;

    public SparkMaxBrushlessEncoderMotor(int canBusId, String motorName, int canCoderId)
    {
        super(canBusId, motorName);
        this.encoderMotor = new CANcoder(canCoderId);
    }

    @Override
    public double getPosition() {
        return this.encoderMotor.getPosition().getValueAsDouble();
    }

    @Override
    public double getAngle() {
        return this.getRadian() / Math.PI * 180;
    }

    @Override
    public double getRadian() {
        double absoluteRotation = this.encoderMotor.getAbsolutePosition().getValueAsDouble();
        absoluteRotation = absoluteRotation * 2 * Math.PI;
        if (absoluteRotation < Math.PI)
        {
            return absoluteRotation;
        } else {
            return absoluteRotation - 2 * Math.PI;
        }
        //return absoluteRotation * 2 * Math.PI;
    }

    @Override
    public void holdPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'holdPosition'");
    }

    @Override
    public void setPosition(double position) {
        this.encoderMotor.setPosition(0);
    }
    
    @Override
    public void printToSmartDashboard() {
        SmartDashboard.putString(this.getName() + " EncoderMotor", "Angle: " + this.getAngle() + "; Speed: " + this.getSpeed());
    }
}
