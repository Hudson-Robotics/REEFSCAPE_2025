package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.swivel;

import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Interfaces.Motors.Motor;

public class swivelL extends SubsystemBase implements swivel{
    final Motor swivelMotor;
    private XboxController controller;

    public swivelL(XboxController controller, Motor  swivelMotor) {
        this.controller = controller;
        this.swivelMotor = swivelMotor;
        this.swivelMotor.enableBrake();
    }

    @Override
    public void periodic() {
        double speedX = this.controller.getLeftX();
        this.swivelMotor.setSpeed(speedX);
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
}
