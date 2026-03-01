package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConsts;
import frc.robot.LimelightHelpers;

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
    // updateOdometry();

    double tx = LimelightHelpers.getTX(""); // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY(""); // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA(""); // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?
    double targetId = NetworkTableInstance.getDefault().getTable(limeLightName).getEntry("tid").getDouble(0);

    double txnc = LimelightHelpers.getTXNC(""); // Horizontal offset from principal pixel/point to target in degrees
    double tync = LimelightHelpers.getTYNC(""); // Vertical offset from principal pixel/point to target in degrees

    // System.out.println("tx" + tx);
    // System.out.println("has target" + hasTarget);
    // System.out.println("ty" + ty);

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

}