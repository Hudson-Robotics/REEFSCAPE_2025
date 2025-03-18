package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.interIntake;

import frc.robot.Interfaces.Motors.Motor;

public class Intake extends SubsystemBase implements interIntake {
    final Motor intakeMotor1;
    final Motor intakeMotor2;

    public Intake(Motor intakeMotor1, Motor intakeMotor2) { 
        this.intakeMotor1 = intakeMotor1;
        this.intakeMotor2 = intakeMotor2;
    }
 
    @ Override
     public void periodic() {
        this.intakeMotor1.printToSmartDashboard();
        this.intakeMotor2.printToSmartDashboard();
     }

    @Override
    public void intake() {
        this.setSpeed(.1);
    }
    @Override
    public void outtake() {
        this.setSpeed(-.1);
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
