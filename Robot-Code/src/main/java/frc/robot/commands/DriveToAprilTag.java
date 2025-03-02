package frc.robot.commands;

import javax.naming.LimitExceededException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;

public class DriveToAprilTag extends Command {
     private final  SwerveDrive swerveDrive;
    private final Limelight limelight;
    private static final double KP_ROTATION = 0.02;
    private static final double KP_FORWARD = .1;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveToAprilTag(SwerveDrive subsystem, Limelight limelight) {
    swerveDrive = subsystem;
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        if (limelight.hasTarget()) {
            double tx = limelight.getTx();
            double ta = limelight.getTa();

            double rotation = KP_ROTATION * tx;
            double forward = KP_FORWARD * (1.0 - ta);

            swerveDrive.drive(forward, 0, -rotation);
        }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  } 
}
