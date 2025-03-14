package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;
import java.util.function.DoubleSupplier;

public class MainCommands {

  private MainCommands() {}

  public static Command stopIntake(Intake intake) {
    return runOnce(
        () -> {
          intake.setSpeed(0);
        },
        intake);
  }

  public static Command runIntake(Intake intake) {
    return runOnce(
        () -> {
          intake.setSpeed(.25); // .25
        },
        intake);
  }

  public static Command runOutake(Intake intake) {
    return runOnce(
        () -> {
          intake.setSpeed(-1);
        },
        intake);
  }

  public static Command runOutakeSlow(Intake intake) {
    return runOnce(
        () -> {
          intake.setSpeed(-.25);
        },
        intake);
  }

  public static Command setWristPerpendicular(Wrist wrist) {
    return sequence(
        runOnce(
            () -> {
              wrist.setPosition(WristConstants.WRIST_PERPENDICULAR_POSITION);
            },
            wrist));
  }

  public static Command setIntakePosition(Wrist wrist, Elevator elevator) {
    return sequence(
        runOnce(
            () -> {
              wrist.setPosition(WristConstants.WRIST_PERPENDICULAR_POSITION);
            },
            wrist),
        waitSeconds(.25),
        runOnce(
            () -> {
              elevator.setPosition(ElevatorConstants.ELEVATOR_INTAKE);
            },
            elevator),
        waitSeconds(0.25),
        runOnce(
            () -> {
              wrist.setPosition(WristConstants.WRIST_INTAKE_POSITION);
            },
            wrist));
  }

  // sets wrist and elevator to 0
  public static Command stow(Wrist wrist, Elevator elevator) {
    return sequence(
        runOnce(
            () -> {
              wrist.setPosition(WristConstants.WRIST_PERPENDICULAR_POSITION);
            },
            wrist),
        waitSeconds(.25),
        runOnce(
            () -> {
              elevator.setPosition(0);
            },
            elevator),
        waitSeconds(.25),
        runOnce(
            () -> {
              wrist.setPosition(0);
            },
            wrist));
  }

  public static Command setClimberUp(Climber climber) {
    return runOnce(
        () -> {
          climber.setSpeed(1);
        },
        climber);
  }

  public static Command setClimberDown(Climber climber) {
    return runOnce(
        () -> {
          climber.setSpeed(-1);
        },
        climber);
  }

  public static Command stopClimber(Climber climber) {
    return runOnce(
        () -> {
          climber.setSpeed(0);
        },
        climber);
  }

  public static Command setElevatorPosition(
      Wrist wrist, Elevator elevator, double elevatorPosition, double wristPosition) {
    return sequence(
        runOnce(
            () -> {
              wrist.setPosition(WristConstants.WRIST_PERPENDICULAR_POSITION);
            },
            wrist),
        waitSeconds(.25),
        runOnce(
            () -> {
              elevator.setPosition(elevatorPosition);
            },
            elevator),
        waitSeconds(
            .25), // og .5 - changed it to 1 for now due to the wrist getting stuck while going down
        runOnce(
            () -> {
              wrist.setPosition(wristPosition);
            },
            wrist));
  }

  // changes all setpoints of the elevator
  public static Command changeElevatorSetpoint(Elevator elevator, double offset) {
    return runOnce(
        () -> {
          elevator.setElevatorSetpoint(offset);
        },
        elevator);
  }

  public static Command changeWristSetpoint(Wrist wrist, double offset) {
    return runOnce(
        () -> {
          wrist.setWristSetpoint(offset);
        },
        wrist);
  }

  // public static Command moveElevator(double position, Elevator elevator) { // testing
  //   return run(() -> elevator.setPosition(position)).until(() -> elevator.atSetpoint());
  // }

  // public static Command moveWrist(double position, Wrist wrist) { // testing
  //   return run(() -> wrist.setPosition(position)).until(() -> wrist.atSetpoint());
  // }

  public static Command setElevatorVoltage(Elevator elevator, DoubleSupplier ySupplier) {
    return runOnce(
        () -> {
          double speed = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.1);
          elevator.setSpeed(speed);
        },
        elevator);
  }

  public static Command stopElevator(Elevator elevator) {
    return runOnce(
        () -> {
          elevator.setSpeed(0);
        },
        elevator);
  }
}
