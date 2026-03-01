// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Subsystems.m_ballTunnel;
import static frc.robot.Subsystems.m_shooter;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public static final Pigeon2 m_gyro = new Pigeon2(TunerConstants.kPigeonId, TunerConstants.kCANBus);

  private final boolean kUseLimelight = true;

  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      RobotContainer.m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          RobotContainer.drivetrain.getState().ModulePositions[0],
          RobotContainer.drivetrain.getState().ModulePositions[1],
          RobotContainer.drivetrain.getState().ModulePositions[2],
          RobotContainer.drivetrain.getState().ModulePositions[3]
      },
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public Robot() {

    m_robotContainer = new RobotContainer();
    /*
     * // 2. Get the initial gyro angle (e.g., from your gyroscope sensor)
     * Rotation2d initialGyroAngle = m_gyro.getRotation2d();
     * 
     * // 3. Get the initial swerve module positions (e.g., from your encoder
     * readings)
     * SwerveModulePosition[] initialModulePositions = new SwerveModulePosition[] {
     * new SwerveModulePosition(0.0, new Rotation2d()), // Front Left (distance,
     * angle)
     * new SwerveModulePosition(0.0, new Rotation2d()), // Front Right
     * new SwerveModulePosition(0.0, new Rotation2d()), // Back Left
     * new SwerveModulePosition(0.0, new Rotation2d()) // Back Right
     * };
     * 
     * // 4. Define the initial pose (optional, defaults to origin if not provided)
     * Pose2d initialPose = new Pose2d(0.0, 0.0, new Rotation2d());
     * 
     * // 6. Create the SwerveDrivePoseEstimator object
     * m_poseEstimator = new SwerveDrivePoseEstimator(
     * RobotContainer.m_kinematics,
     * initialGyroAngle,
     * initialModulePositions,
     * initialPose);
     */
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Limelight Localization Code
    if (kUseLimelight) {
      /*
       * var driveState = RobotContainer.drivetrain.getState();
       * double omegaRps = Units.radiansToRotations(
       * driveState.Speeds.omegaRadiansPerSecond);
       * 
       * // FRONT LIMELIGHT — MEGATAG 1
       * var llMeasurement1 =
       * LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");
       * 
       * if (llMeasurement1 != null && llMeasurement1.tagCount > 0
       * && Math.abs(omegaRps) < 2.0) {
       * RobotContainer.drivetrain.addVisionMeasurement(
       * llMeasurement1.pose,
       * llMeasurement1.timestampSeconds);
       * }
       * 
       * // BACK LIMELIGHT — MEGATAG 1
       * var llMeasurement2 =
       * LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");
       * 
       * if (llMeasurement2 != null && llMeasurement2.tagCount > 0
       * && Math.abs(omegaRps) < 2.0) {
       * RobotContainer.drivetrain.addVisionMeasurement(
       * llMeasurement2.pose,
       * llMeasurement2.timestampSeconds);
       */
      updateOdometry();
      // }
    }
    SmartDashboard.putNumber("X", m_robotContainer.drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("Y", m_robotContainer.drivetrain.getState().Pose.getY());
    SmartDashboard.putNumber("Rot", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Shooter velocity", m_shooter.getVelocity());
    SmartDashboard.putNumber("ball velocity", m_ballTunnel.getVelocity());
    RobotContainer.m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
    Subsystems.m_limelight.getAprilTag();
    Subsystems.m_limelight2.getAprilTag();

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
      DogLog.log("selected auto", m_autonomousCommand.getName());
    } else {
      // No auto selected
      DogLog.log("selected auto", "none");
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
    // System.out.println("leftX: " + Controls.joystick.getLeftX());
    // System.out.println("rightX: " + Controls.joystick.getRightX());
    // System.out.println("rightY: " + Controls.joystick.getRightY());
    try {
      System.out.println("Drive Request: " + RobotContainer.drivetrain.getDefaultCommand());
    } catch (Exception e) {
      System.out.println("gayBoy: " + e);
    }
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

  public void updateOdometry() {
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            RobotContainer.drivetrain.getState().ModulePositions[0],
            RobotContainer.drivetrain.getState().ModulePositions[1],
            RobotContainer.drivetrain.getState().ModulePositions[2],
            RobotContainer.drivetrain.getState().ModulePositions[3]
        });

    boolean useMegaTag2 = true; // set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if (useMegaTag2 == false) {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");

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
      LimelightHelpers.SetRobotOrientation("limelight-back",
          m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
      if (Math.abs(m_gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater
                                                                                // than 720 degrees per second, ignore
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

}
