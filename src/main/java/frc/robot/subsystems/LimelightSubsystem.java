// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import static frc.robot.Robot.m_poseEstimator;
import static frc.robot.Robot.m_gyro;
import static frc.robot.RobotContainer.backLeft;
import static frc.robot.RobotContainer.frontLeft;
import static frc.robot.RobotContainer.backRight;
import static frc.robot.RobotContainer.frontRight;

public class LimelightSubsystem extends SubsystemBase {
    public LimelightHelpers helper = new LimelightHelpers();

    /** Creates a new LimelightSubsystem. */
    public LimelightSubsystem() {

    }

    @Override
    public void periodic() {
        updateOdometry();

        double tx = LimelightHelpers.getTX(""); // Horizontal offset from crosshair to target in degrees
        double ty = LimelightHelpers.getTY(""); // Vertical offset from crosshair to target in degrees
        double ta = LimelightHelpers.getTA(""); // Target area (0% to 100% of image)
        boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

        double txnc = LimelightHelpers.getTXNC(""); // Horizontal offset from principal pixel/point to target in degrees
        double tync = LimelightHelpers.getTYNC(""); // Vertical offset from principal pixel/point to target in degrees

        System.out.println("tx" + tx);
        System.out.println("has target" + hasTarget);
        System.out.println("ty" + ty);
    }

    public void updateOdometry() {

        m_poseEstimator.update(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {

                        frontLeft,
                        frontRight,
                        backLeft,
                        backRight
                });

        boolean useMegaTag2 = true; // set to false to use MegaTag1
        boolean doRejectUpdate = false;
        if (useMegaTag2 == false) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

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
            LimelightHelpers.SetRobotOrientation("limelight",
                    m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
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
}
