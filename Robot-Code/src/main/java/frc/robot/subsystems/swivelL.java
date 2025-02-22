package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.swivel;
import frc.robot.Constants.SparkMaxIDs;
import javax.sound.sampled.Port;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;


public class swivelL extends SubsystemBase implements swivel{
    final SparkMax swivelMotor;
    private XboxController controller; 
public swivelL(XboxController controller) {
    this.controller = controller;
    this.swivelMotor = new SparkMax(SparkMaxIDs.SWIVEL_MOTOR_UP, MotorType.kBrushed);
}

@Override
public void periodic() {
    double speedX = this.controller.getLeftX();
    swivelMotor.set(.05);
    swivelMotor.set(-.05);
}

@Override
public void up() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'up'");
}

@Override
public void down() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'down'");
}

@Override
public void stop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
}
}
