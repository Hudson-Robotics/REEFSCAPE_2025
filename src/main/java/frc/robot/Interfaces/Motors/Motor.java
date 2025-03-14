package frc.robot.Interfaces.Motors;

public interface Motor {
  public void setSpeed(double speed);

  public double getSpeed();

  public String getName();

  public void printToSmartDashboard();
}
