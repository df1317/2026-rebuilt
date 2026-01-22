package frc.robot.subsystems.swervedrive;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swervedrive.avoidance.FieldZones;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

import java.io.InputStream;
import java.util.function.DoubleSupplier;

public class OurSwerveInputStream extends SwerveInputStream {

  private SwerveDrive swerveDrive;

  public OurSwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    super(drive, x, y, rot);
    this.swerveDrive = swerveDrive;
  }

  @Override
  public ChassisSpeeds get(){
    Rotation2d currentHeading = swerveDrive.getOdometryHeading();
    Translation2d relativeTrl    = FieldZones.HUB_POSE.relativeTo(swerveDrive.getPose()).getTranslation();
    Rotation2d    target         = new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(currentHeading);
    DogLog.log("HeadingSetpoint", target.getRadians());
    return super.get();
  }
}
