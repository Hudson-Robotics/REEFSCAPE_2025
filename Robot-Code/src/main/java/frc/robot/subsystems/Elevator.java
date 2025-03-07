package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Interfaces.interElevator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Interfaces.Motors.Motor;

public class Elevator extends SubsystemBase implements interElevator{

    final Motor eleMotor1;
    final Motor eleMotor2;   
    private XboxController controller; 
    
    public Elevator(XboxController controller, Motor eleMotor1, Motor eleMotor2) {
        controller = new XboxController(1);
        this.controller = controller;

        this.eleMotor1 = eleMotor1; //might be more useful to name this as left and right motors
        this.eleMotor2 = eleMotor2;
    }

    @Override
    public void periodic() {
        // Get the state of the RB button (boolean)
        boolean RBButtonPressed = controller.getRightBumper(); // Look into xbox controller documentation frc
        SmartDashboard.putBoolean("RB Button Pressed", RBButtonPressed);
                //to go down-
                boolean LBButtonPressed = controller.getLeftBumper();
                SmartDashboard.putBoolean("LB Button Pressed", LBButtonPressed);
        //going up+
        if (RBButtonPressed) {
            this.raise();
        } else if (LBButtonPressed) {
            this.drop();
        } else {
            this.stop();
        }
    }

    //the elevators are mirrored so they might have to be inverted
    @Override
    public void raise() {
        this.setSpeed(.2);
    }

    @Override
    public void drop() {
        this.setSpeed(-.2);
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



