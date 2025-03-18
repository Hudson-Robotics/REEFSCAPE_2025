package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.swivel;

import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Interfaces.Motors.MotorWithEncoder;

public class swivelL extends SubsystemBase implements swivel{
    final MotorWithEncoder swivelMotor;

    public swivelL(MotorWithEncoder  swivelMotor) {
        this.swivelMotor = swivelMotor;
        this.swivelMotor.enableBrake();
    }

    @Override
    public void periodic() {
        this.swivelMotor.printToSmartDashboard();
    }

    @Override
    public void up() {
        this.swivelMotor.setSpeed(.05);
    }

    @Override
    public void down() {
        this.swivelMotor.setSpeed(-.05);
    }

    @Override
    public void stop() {
        this.swivelMotor.setSpeed(0);
    }

    @Override
    public void setSpeed(double speed) {
        this.swivelMotor.setSpeed(speed);
    }

    @Override
    public void setAngle(double angle) {
        double encoderPositition = angle; //need to converted
    }
}
