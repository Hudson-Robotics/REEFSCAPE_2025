package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Interfaces.interElevator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Interfaces.Motors.Motor;
import frc.robot.Interfaces.Motors.MotorWithEncoder;

public class Elevator extends SubsystemBase implements interElevator{

    final MotorWithEncoder eleMotor1;
    final MotorWithEncoder eleMotor2;   
    
    public Elevator(MotorWithEncoder eleMotor1, MotorWithEncoder eleMotor2) {
        this.eleMotor1 = eleMotor1; //might be more useful to name this as left and right motors
        this.eleMotor2 = eleMotor2;
    }

    @Override
    public void periodic() {
        this.eleMotor1.printToSmartDashboard();
        this.eleMotor2.printToSmartDashboard();
        Logger.recordOutput("Elevator 1", this.eleMotor1.getPosition());
        Logger.recordOutput("Elevator 2", this.eleMotor2.getPosition());
    }

    //the elevators are mirrored so they might have to be inverted
    @Override
    public void raise() {
        this.setSpeed(.1);
    }

    @Override
    public void drop() {
        this.setSpeed(-.1);
    }

    @Override
    public void stop() {
        this.setSpeed(0);
    }

    @Override
    public void setSpeed(double speed) {
        this.eleMotor1.setSpeed(speed);
        this.eleMotor2.setSpeed(speed);
    }

    @Override
    public void setHeight(double height) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setHeight'");
    }
}



