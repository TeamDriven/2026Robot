// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Subsystems.m_angleController;
import static frc.robot.Subsystems.m_ballTunnel;
import static frc.robot.Subsystems.m_intakeActuation;
import static frc.robot.Subsystems.m_intakeRollers;
import static frc.robot.Subsystems.m_limelight;
import static frc.robot.Subsystems.m_shooter;

import java.util.Optional;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  private final boolean kUseLimelight = true;
  public static Alliance alliance;


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
    SmartDashboard.putNumber("shooter velo", m_shooter.getVelocity());
    SmartDashboard.putNumber("avg distance lime", m_limelight.getMegaTag2().avgTagDist);
    SmartDashboard.putNumber("xPose", RobotContainer.drivetrain.getPose().getX());
    SmartDashboard.putNumber("yPose", RobotContainer.drivetrain.getPose().getY());
    m_robotContainer.robotPeriodic();

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
    /**
     * Axis: Pitch (Lateral), Roll (Longitudinal), Yaw (Vertical).
      Movement: Pitch (Up/Down), Roll (Side-to-Side/Bank), Yaw (Left/Right).
     */

    Subsystems.m_limelight.getAprilTag();

  }

  @Override
  public void disabledInit() {
    m_intakeActuation.setCoastMode();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if(ally.isPresent()) alliance = ally.get();
    
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
    m_intakeRollers.stopIntakeMotor();
    m_shooter.stopMotors();
    m_ballTunnel.stopBallTunnel();
    m_intakeActuation.setPosition(1.3);
    m_angleController.resetAngleToZeroCommand();
  }

  @Override
  public void teleopInit() {
        Optional<Alliance> ally = DriverStation.getAlliance();
    if(ally.isPresent()){
      if(ally.get() == Alliance.Red) {
        System.out.println("Red");
      }
      else if(ally.get() == Alliance.Blue) {
        System.out.println("Blue");
      }
      alliance = ally.get();
    } 
 
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_angleController.setPosition(0.2);
  }

  @Override
  public void teleopPeriodic() {
    // try {
    //   System.out.println("Drive Request: " + RobotContainer.drivetrain.getDefaultCommand());
    // } catch (Exception e) {
    //   System.out.println("error: " + e);
    // }
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
    boolean useMegaTag2 = true; // set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if (useMegaTag2 == false) {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelight.getLimelightName());

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
        RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
       RobotContainer.drivetrain.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    } else if (useMegaTag2 == true) {
      LimelightHelpers.SetRobotOrientation(m_limelight.getLimelightName(),
         RobotContainer.drivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelight.getLimelightName());

      if (Math.abs(RobotContainer.drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater
                                                                                // than 720 degrees per second, ignore
                                                                                // vision updates
      {
        doRejectUpdate = true;
      }
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999)); //technically just need to set it once if you trust it; can change the dynamic (n1, n2) for confidence, like how far they are, which ones, etc
       RobotContainer.drivetrain.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds); //can you feed the std devs
      }
    }
  }

}
