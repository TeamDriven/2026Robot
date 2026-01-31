package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConsts;
import frc.robot.LimelightHelpers;

import static frc.robot.Robot.m_poseEstimator;
import static frc.robot.Robot.m_gyro;
import static frc.robot.RobotContainer.backLeft;
import static frc.robot.RobotContainer.frontLeft;
import static frc.robot.RobotContainer.backRight;

import static frc.robot.RobotContainer.frontRight;

import java.util.function.DoubleSupplier;

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

  public void updateOdometry() {

    m_poseEstimator.update(
        new Rotation2d(m_gyro.getRotation2d().getRadians()),
        new SwerveModulePosition[] {

            frontLeft,
            frontRight,
            backLeft,
            backRight
        });

    boolean useMegaTag2 = true; // set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if (useMegaTag2 == false) {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limeLightName);
      // LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");

      if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        if (mt1.rawFiducials[0].ambiguity > .7) {
          doRejectUpdate = true;
        }
        if (mt1.rawFiducials[0].distToCamera > 3) {
          doRejectUpdate = true;
        }
      }
      if (mt1.tagCount == 0) {
        doRejectUpdate = true;
      }

      if (!doRejectUpdate) {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    } else if (useMegaTag2 == true) {
      LimelightHelpers.SetRobotOrientation(limeLightName,
          m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightName);
      // LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
      if (Math.abs(m_gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is
                                                                                // greater
                                                                                // than 720 degrees per second,
                                                                                // ignore
                                                                                // vision updates
      {
        doRejectUpdate = true;
      }
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }

  public double limelight_aim_proportional()
  {
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

    public double getAprilTag(){
        SmartDashboard.putNumber("April Tag Number", LimelightHelpers.getFiducialID(VisionConsts.LIMELIGHT_NAME));
        return LimelightHelpers.getFiducialID(VisionConsts.LIMELIGHT_NAME);
    }

    public double getAprilTagHeight(){
      double aprilTag = getAprilTag();
        if (aprilTag == 1 || aprilTag == 6 || aprilTag == 7 ||aprilTag == 12) {
          SmartDashboard.putString("Tag Reading", "Trench");
          return VisionConsts.UP_TO_TRENCH_TAG;
        } 
        if (aprilTag == 13 ||aprilTag == 14) {
          SmartDashboard.putString("Tag Reading", "Corral");
          return VisionConsts.UP_TO_CORRAL_TAG;
        } 
        if (aprilTag == 2 ||aprilTag == 3 || aprilTag == 4 || aprilTag == 5 || aprilTag == 8 ||aprilTag == 9 || aprilTag == 10 ||aprilTag == 11) {
          SmartDashboard.putString("Tag Reading", "Hub");
          return VisionConsts.UP_TO_HUB_TAG;
        }

  return -1;

    }

}