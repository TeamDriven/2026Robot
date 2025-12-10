// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public static final Pigeon2 m_gyro = new Pigeon2(TunerConstants.kPigeonId);
  public static SwerveDrivePoseEstimator m_poseEstimator;

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
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Pose2d pose = m_poseEstimator.getEstimatedPosition();
    RobotContainer.m_field.setRobotPose(pose);
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
      m_autonomousCommand.schedule();
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
