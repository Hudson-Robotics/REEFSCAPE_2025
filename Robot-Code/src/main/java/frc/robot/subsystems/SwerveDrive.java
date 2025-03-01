//no sue plz :D

package frc.robot.subsystems;

import frc.robot.subsystems.helper.SwerveModule;
//import com.ctre.phoenix6.swerve.SwerveModule;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import static edu.wpi.first.units.Units.Newton;

import java.util.Random;

import javax.xml.xpath.XPathVariableResolver;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Interfaces.DriveTrain;
import frc.robot.subsystems.helper.SwerveModule;

import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class SwerveDrive extends SubsystemBase  implements DriveTrain {
  
  

  //xbox
  private XboxController controller; 
  SwerveModule frontLeftModule;
  SwerveModule frontRightModule;
  SwerveModule backLeftModule;
  SwerveModule backRightModule;

    Translation2d frontLeftLocation = new Translation2d(-.381, .381); //.381 is half of .762 m which is 30 inches in freedom units
    Translation2d frontRightsLocation = new Translation2d(.381, .381); //.381 is half of .762 m which is 30 inches in freedom unitS
    Translation2d backLeftLocation = new Translation2d(-.381, -.381); //.381 is half of .762 m which is 30 inches in freedom units
    Translation2d backRightLocation = new Translation2d(.381, -.381); //.381 is half of .762 m which is 30 inches in freedom units

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightsLocation, backLeftLocation, backRightLocation);

    public SwerveDrive(SwerveModule flm, SwerveModule frm, SwerveModule blm, SwerveModule brm) { 
      //AHRS x = new AHRS(SPI.Port.kMXP);

      controller = new XboxController(1);

         this.frontLeftModule = flm;
         this.frontRightModule = frm;
         this.backLeftModule = blm;
         this.backRightModule = brm;
     }
     /**
      * Example command factory method.
      *
      * @return a command
      */
     public Command exampleMethodCommand() {
       // Inline construction of command goes here.
       // Subsystem::RunOnce implicitly requires `this` subsystem.
       return runOnce(
           () -> {
             /* one-time action goes here */
           });
     }
     /**
      * An example method querying a boolean state of the subsystem (for example, a digital sensor).
      *
      * @return value of some boolean subsystem state, such as a digital sensor.
      */
     public boolean exampleCondition() {
       // Query some boolean state, such as a digital sensor.
       return false;
     }
   
     @Override
     public void periodic() {
      double speedX = this.controller.getLeftX();
      double speedY = this.controller.getLeftY();
      double rotate = this.controller.getRightY();
      // maybe add slewrates....

      var SwerveModuleState = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rotate, new Rotation2d(0)));
      frontLeftModule.setDesiredState(SwerveModuleState[0]);
      frontRightModule.setDesiredState(SwerveModuleState[1]);
      backLeftModule.setDesiredState(SwerveModuleState[2]);
      backRightModule.setDesiredState(SwerveModuleState[3]);

      SmartDashboard.putString("Front Left", SwerveModuleState[0].toString());
      SmartDashboard.putString("Front Right", SwerveModuleState[1].toString());
      SmartDashboard.putString("Back Left", SwerveModuleState[2].toString());
      SmartDashboard.putString("Back Right", SwerveModuleState[3].toString());
    }
     

    


      
      
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

@Override
public void drive(final double xspeed, final double yspeed, final double rotation) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'drive go'");
  }
}

