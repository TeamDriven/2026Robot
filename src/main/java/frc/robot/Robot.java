// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public static final Pigeon2 m_gyro = new Pigeon2(TunerConstants.kPigeonId);
  public static SwerveDrivePoseEstimator m_poseEstimator;
  private final boolean kUseLimelight = true;

  public Robot() {
    m_robotContainer = new RobotContainer();

    // 2. Get the initial gyro angle (e.g., from your gyroscope sensor)
    Rotation2d initialGyroAngle = m_gyro.getRotation2d();

    // 3. Get the initial swerve module positions (e.g., from your encoder readings)
    SwerveModulePosition[] initialModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(0.0, new Rotation2d()), // Front Left (distance, angle)
        new SwerveModulePosition(0.0, new Rotation2d()), // Front Right
        new SwerveModulePosition(0.0, new Rotation2d()), // Back Left
        new SwerveModulePosition(0.0, new Rotation2d()) // Back Right
    };

    // 4. Define the initial pose (optional, defaults to origin if not provided)
    Pose2d initialPose = new Pose2d(0.0, 0.0, new Rotation2d());

    // 6. Create the SwerveDrivePoseEstimator object
    m_poseEstimator = new SwerveDrivePoseEstimator(
        RobotContainer.m_kinematics,
        initialGyroAngle,
        initialModulePositions,
        initialPose);

    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
    DogLog.setPdh(RobotContainer.m_pdh);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // drivetrain.updateOdometry();
    // Pose2d pose = m_poseEstimator.getEstimatedPosition();
    // // Pose2d pose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    // // SmartDashboard.putNumber("limelight
    // TX",LimelightHelpers.getTX("limelight"));
    // // SmartDashboard.putNumber("limelight
    // TY",LimelightHelpers.getTY("limelight"));
    // // SmartDashboard.putNumber("limelight
    // TA",LimelightHelpers.getTA("limelight"));
    // System.out.println("X Pos: " +
    // m_poseEstimator.getEstimatedPosition().getX());
    // System.out.println("Y Pos: " +
    // m_poseEstimator.getEstimatedPosition().getY());
    // System.out.println("Rot: " +
    // m_poseEstimator.getEstimatedPosition().getRotation());
    // m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight-front", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
      if (llMeasurement1 != null && llMeasurement1.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement1.pose, llMeasurement1.timestampSeconds);
      }

      // LimelightHelpers.SetRobotOrientation("limelight-back", headingDeg, 0, 0, 0,
      // 0, 0);
      // var llMeasurement2 =
      // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
      // if (llMeasurement2 != null && llMeasurement2.tagCount > 0 &&
      // Math.abs(omegaRps) < 2.0) {
      // m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement2.pose,
      // llMeasurement2.timestampSeconds);
      // }
    }
    // SmartDashboard.putNumber("X", m_robotContainer.drivetrain.getState().Pose.getX());
    // SmartDashboard.putNumber("Y", m_robotContainer.drivetrain.getState().Pose.getY());
    // SmartDashboard.putNumber("Rot", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
    RobotContainer.m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }

}
