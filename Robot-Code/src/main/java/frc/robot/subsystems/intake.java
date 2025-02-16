package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.Intake;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.SparkMaxIDs;

public class intake extends SubsystemBase implements Intake {
  final SparkMax intakeMotor1;
  final SparkMax intakeMotor2;

  private XboxController controller; 
 public intake() { 
    controller = new XboxController(1);
    intakeMotor1 = new SparkMax(SparkMaxIDs.INTAKE_MOTOR_ONE, MotorType.kBrushed);
    intakeMotor2 = new SparkMax(SparkMaxIDs.INTAKE_MOTOR_TWO, MotorType.kBrushed); 
 }
 
    @ Override
     public void periodic() {
      // Get the state of the A button (boolean)
     boolean xButtonPressed = controller.getAButton();
      SmartDashboard.putBoolean("X Button Pressed", xButtonPressed); 
      SmartDashboard.putNumber("upMotor1", 1); 
      intakeMotor1.set(1); 
      SmartDashboard.putNumber("upMotor2", 1); 
      intakeMotor2.set(1);   
     }

    @Override
    public void intake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'intake'");
    }
    @Override
    public void outtake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'outtake'");
    }
    @Override
    public void setSpeed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSpeed'");
    }
    @Override
    public boolean hasCoral() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'hasCoral'");
    }

    
}
