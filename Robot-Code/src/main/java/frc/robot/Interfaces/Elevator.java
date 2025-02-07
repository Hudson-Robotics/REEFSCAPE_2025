package frc.robot.Interfaces;

public interface Elevator {
    void raise();
    void drop();
    void stop();
    void setSpeed(double speed);
    void setHeight(double height);
}
