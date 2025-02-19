package frc.robot.subsystems;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.Climb;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.SparkMaxIDs;


public class Climber extends SubsystemBase implements Climb  {
 final SparkMax upMotor1;
 final SparkMax UpMotor2;

 private XboxController controller; 
 public Climber() { 
    controller = new XboxController(2);
    upMotor1 = new SparkMax(SparkMaxIDs.UP_MOTOR_CLIMB_ONE, MotorType.kBrushed);
    UpMotor2 = new SparkMax(SparkMaxIDs.UP_MOTOR_CLIMB_TWO, MotorType.kBrushed);
}

    @Override
     public void periodic() {
      // Get the state of the A button (boolean)
     boolean xButtonPressed = controller.getAButton();
      SmartDashboard.putBoolean("X Button Pressed", xButtonPressed); 
      SmartDashboard.putNumber("upMotor1", 1); 
      upMotor1.set(1); 
      SmartDashboard.putNumber("upMotor2", 1); 
      UpMotor2.set(1);   
     }

    @Override
    public void climb(double speed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'climb'");
    }
}