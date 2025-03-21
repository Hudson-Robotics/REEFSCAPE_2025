package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.Climb;

import frc.robot.Interfaces.Motors.Motor;

public class Climber extends SubsystemBase implements Climb  {
 final Motor upMotor1;
 final Motor UpMotor2;

 final DoubleSolenoid leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 13);
 final DoubleSolenoid rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 15);
 boolean isItClamped = false;

 public Climber(Motor upMotor1, Motor upMotor2) { 
    this.upMotor1 = upMotor1;
    this.UpMotor2 = upMotor2;
}

    @Override
     public void periodic() {
        this.upMotor1.printToSmartDashboard();
        this.UpMotor2.printToSmartDashboard();
     }

    @Override
    public void climb(double speed) {
        this.upMotor1.setSpeed(speed);
        this.UpMotor2.setSpeed(speed);
    }

    @Override
    public void clamp() {
        if(!isItClamped)
        {
            leftSolenoid.toggle();
            rightSolenoid.toggle(); 
            isItClamped = true;
        }
    }

    @Override
    public void unclamp() {
        if(isItClamped) {
            leftSolenoid.toggle();
            rightSolenoid.toggle();
            isItClamped = false;
        }
    }

    
}