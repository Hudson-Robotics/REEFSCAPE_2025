package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.swivel;

import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Interfaces.Motors.Motor;
import frc.robot.Interfaces.Motors.MotorWithEncoder;

public class swivelL extends SubsystemBase implements swivel{
    final MotorWithEncoder swivelMotor;
    private XboxController controller;

    public swivelL(XboxController controller, MotorWithEncoder  swivelMotor) {
        this.controller = controller;
        this.swivelMotor = swivelMotor;
        this.swivelMotor.enableBrake();
    }

    @Override
    public void periodic() {
        double speedX = this.controller.getLeftX();
        double threshold = .04;
        speedX = Math.abs(speedX) < threshold ? 0.0 : speedX;
        if(speedX == 0.0)
        {
            //this.swivelMotor.holdPosition(); // this needs to be fixed but a different branch problem
        } else {
            this.swivelMotor.setSpeed(speedX * .1); // wanna scale it down for safety
            this.swivelMotor.printToSmartDashboard();
        }
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
    public void setAngle(double angle) {
        double encoderPositition = angle; //need to converted
    }
}
