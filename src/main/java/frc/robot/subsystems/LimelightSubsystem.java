package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConsts;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConst;


public class LimelightSubsystem extends SubsystemBase {
  public LimelightHelpers helper = new LimelightHelpers();
  private final String limeLightName;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem(String name) {
    limeLightName = name;
  }

  @Override
  public void periodic() {
  }

  public double limelight_aim_proportional() {
    double kP = .035;
    double targetingAngularVelocity = LimelightHelpers.getTX(limeLightName) * kP * -DrivetrainConst.MaxAngularRate;
    // convert to radians per second for our drive method

    return targetingAngularVelocity;
  }

  public double limelight_range_proportional() {
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY(limeLightName) * kP * -DrivetrainConst.MaxSpeed;
    return targetingForwardSpeed;
  }

  public double getAprilTag() {
    SmartDashboard.putNumber("April Tag Number" + limeLightName, LimelightHelpers.getFiducialID(limeLightName));
    return LimelightHelpers.getFiducialID(VisionConsts.LIMELIGHT_NAME);
  }

  public double getAprilTagHeight() {
    double aprilTag = getAprilTag();
    if (aprilTag == 1 || aprilTag == 6 || aprilTag == 7 || aprilTag == 12) {
      SmartDashboard.putString("Tag Reading", "Trench");
      return VisionConsts.UP_TO_TRENCH_TAG;
    }
    if (aprilTag == 13 || aprilTag == 14) {
      SmartDashboard.putString("Tag Reading", "Corral");
      return VisionConsts.UP_TO_CORRAL_TAG;
    }
    if (aprilTag == 2 || aprilTag == 3 || aprilTag == 4 || aprilTag == 5 || aprilTag == 8 || aprilTag == 9
        || aprilTag == 10 || aprilTag == 11) {
      SmartDashboard.putString("Tag Reading", "Hub");
      return VisionConsts.UP_TO_HUB_TAG;
    }

    return -1;

  }

   public LimelightHelpers.PoseEstimate getMegaTag2(String LIMELIGHT_NAME) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
  }

  public LimelightHelpers.PoseEstimate getMegaTag1(String LIMELIGHT_NAME) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);
  }

}