



package frc.robot.subsystems;
import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveGeneral;
import static edu.wpi.first.units.Units.Meter;
public class SwerveSubsystem extends SubsystemBase {
  double maximumSpeed = Units.feetToMeters(4.5);
File directory = new File(Filesystem.getDeployDirectory(),"swerve");
SwerveDrive  swerveDrive ;
  
  public SwerveSubsystem()  {
    
        try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveDriveGeneral.MAX_SPEED,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                               Meter.of(4)),
                                                                             Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }


  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public SwerveDrive getSwerveDrive() {
  return swerveDrive;
  }

public void driveFieldOriented(ChassisSpeeds velocity){
  driveFieldOriented(velocity);
}

public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
  return run(() -> {
swerveDrive.driveFieldOriented(velocity.get());
  }
  );
}

public void drive (Translation2d translation,double rotation, boolean isFieldrelative){
  swerveDrive.drive(translation,rotation,isFieldrelative,false);
}



public void resetOdometry(Pose2d initialHolonomicPose)
{
  swerveDrive.resetOdometry(initialHolonomicPose);

}

public void setChassisSpeeds(ChassisSpeeds speeds){
  swerveDrive.setChassisSpeeds(speeds);
  swerveDrive.postTrajectory(new Trajectory());
}


public void zeroGyro(){
  swerveDrive.zeroGyro();
}


 private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }



}
