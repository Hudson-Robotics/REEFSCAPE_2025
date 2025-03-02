package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.interIntake;

import frc.robot.Interfaces.Motors.Motor;

public class Intake extends SubsystemBase implements interIntake {
    final Motor intakeMotor1;
    final Motor intakeMotor2;

    private XboxController controller; 
    public Intake(Motor intakeMotor1, Motor intakeMotor2) { 
        controller = new XboxController(0); // todo: need to extract this
        this.intakeMotor1 = intakeMotor1;
        this.intakeMotor2 = intakeMotor2;
    }
 
    @ Override
     public void periodic() {
        // Get the state of the A button (boolean)
        boolean xButtonPressed = controller.getAButton();
        SmartDashboard.putBoolean("X Button Pressed", xButtonPressed);
        boolean bButtonPressed = controller.getBButton();
        SmartDashboard.putBoolean("B Button Pressed", bButtonPressed); 

        if(xButtonPressed){
            this.intake();
        } else if(bButtonPressed) {
            this.outtake();
        } else {
            this.setSpeed(0); // we should probably add this to the interface
        }

        this.intakeMotor1.printToSmartDashboard();
        this.intakeMotor2.printToSmartDashboard();
     }

    @Override
    public void intake() {
        this.setSpeed(.25);
    }
    @Override
    public void outtake() {
        this.setSpeed(-.25);
    }
    @Override
    public void setSpeed(double speed) {
        this.intakeMotor1.setSpeed(-speed);
        this.intakeMotor2.setSpeed(speed);
    }
    @Override
    public boolean hasCoral() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'hasCoral'");
    }

    
}
