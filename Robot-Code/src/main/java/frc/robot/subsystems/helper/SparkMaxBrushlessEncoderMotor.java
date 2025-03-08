package frc.robot.subsystems.helper;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interfaces.Motors.MotorWithEncoder;

public class SparkMaxBrushlessEncoderMotor extends SparkMaxBrushlessMotor implements MotorWithEncoder{
    private CANcoder encoderMotor;
    private double conversionFactor;

    public SparkMaxBrushlessEncoderMotor(int canBusId, String motorName, boolean useAlternate, double conversionFactor, int encoderCanBusId)
    {
        super(canBusId, motorName);
        // this.encoderMotor = useAlternate ?  super.motor.getAlternateEncoder() : super.motor.getEncoder();
        if(canBusId != -1)
        {
            this.encoderMotor = new CANcoder(encoderCanBusId);
        }
        
        this.conversionFactor = conversionFactor;
    }

    @Override
    public double getPosition() {
        return this.encoderMotor.getPosition().getValueAsDouble();
        //return 0;
    }

    @Override
    public double getAngle() {
        String units = this.encoderMotor.getPosition().getUnits();
        double posittion = this.encoderMotor.getAbsolutePosition().getValueAsDouble();//this.encoderMotor.getPosition().getValueAsDouble();
        double radians = posittion * 2  * Math.PI;
        SmartDashboard.putNumber("getAngle degrees", radians);
        //SmartDashboard.putNumber("encoder Motor ToString", this.encoderMotor.getAbsolutePosition().getValueAsDouble());
        return radians;
    }

    @Override
    public double getRadian() {
        return this.getAngle() / 180 * Math.PI;
    }

    @Override
    public void holdPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'holdPosition'");
    }
    
    // @Override
    // public void printToSmartDashboard() {
    //     SmartDashboard.putString(this.getName(), "Angle " + this.getAngle());
    // }
}
