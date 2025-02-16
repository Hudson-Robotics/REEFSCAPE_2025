package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Interfaces.interElevator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SparkMaxIDs;

public class Elevator extends SubsystemBase implements interElevator{
 private static final boolean RBButtonPressed = false;
     //sparkmax & xbox controller
     final SparkMax eleMotor1;
  final SparkMax eleMotor2;   
 private XboxController controller;

 public Elevator(XboxController controller) {
     this.controller = controller;
     eleMotor1 = new SparkMax(SparkMaxIDs.ELEVATOR_MOTOR_ONE, MotorType.kBrushed);
     eleMotor2 = new SparkMax(SparkMaxIDs.ELEVATOR_MOTOR_TWO, MotorType.kBrushed);
 }

 @Override
 public void periodic() {
     // Get the state of the RB button (boolean)
     boolean RBButtonPressed = controller.getRightBumper(); // Look into xbox controller documentation frc
     SmartDashboard.putBoolean("RB Button Pressed", RBButtonPressed);
     //going up+
     if (RBButtonPressed) {
         SmartDashboard.putNumber("eleMotor1", .05); 
         eleMotor1.set(.05); 
         SmartDashboard.putNumber("eleMotor2", .05); 
         eleMotor2.set(.05); 
     }

     //to go down-
     boolean LBButtonPressed = controller.getLeftBumper();
     SmartDashboard.putBoolean("LB Button Pressed", LBButtonPressed);
     if (LBButtonPressed) {
         SmartDashboard.putNumber("eleMotor1", -.05); 
         eleMotor1.set(-.05); 
         SmartDashboard.putNumber("eleMotor2", -.05); 
         eleMotor2.set(-.05);
     }
 }

 @Override
 public void raise() {
     // TODO Auto-generated method stub
     throw new UnsupportedOperationException("Unimplemented method 'raise'");
 }

 @Override
 public void drop() {
     // TODO Auto-generated method stub
     throw new UnsupportedOperationException("Unimplemented method 'drop'");
 }

 @Override
 public void stop() {
     // TODO Auto-generated method stub
     throw new UnsupportedOperationException("Unimplemented method 'stop'");
 }

 @Override
 public void setSpeed(double speed) {
     // TODO Auto-generated method stub
     throw new UnsupportedOperationException("Unimplemented method 'setSpeed'");
 }

 @Override
 public void setHeight(double height) {
     // TODO Auto-generated method stub
     throw new UnsupportedOperationException("Unimplemented method 'setHeight'");
 }
}



