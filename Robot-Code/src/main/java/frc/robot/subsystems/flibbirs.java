package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Interfaces.interFlibbirs;






public class flibbirs extends SubsystemBase implements interFlibbirs {
    // Define the solenoid
      private XboxController controller; 

    private final DoubleSolenoid exampleDouble = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
    public flibbirs() {
        // Initialize the DoubleSolenoid so it knows where to start.  Not required for single solenoids.
        exampleDouble.set(DoubleSolenoid.Value.kReverse);
    }

    public void extend() {
        exampleDouble.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        exampleDouble.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void squish() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'squish'");
    }

    @Override
    public void unsquish() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'unsquish'");
    }

    @Override
    public void setSpeed(double speed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSpeed'");
    }
}





