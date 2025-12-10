// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.target.pipeline.NeuralClassifier;
import static frc.robot.RobotContainer.m_gyro;
import static frc.robot.Robot.m_poseEstimator;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

public class LimelightSubsystem extends SubsystemBase {

        /** Creates a new LimelightSubsystem. */
        public LimelightSubsystem() {
        }

    @Override
    public void periodic() {
        Limelight limelight = new Limelight("limelight");

        // Set the limelight to use Pipeline LED control, with the Camera offset of 0, and save.
        limelight.getSettings()
                .withLimelightLEDMode(LEDMode.PipelineControl)
                .withCameraOffset(Pose3d.kZero)
                .save();

        // Get target data
        limelight.getLatestResults().ifPresent((LimelightResults result) -> {
                for (NeuralClassifier object : result.targets_Classifier) {
                        // Classifier says its a algae.
                        if (object.className.equals("algae")) {
                                // Check pixel location of algae.
                                if (object.ty > 2 && object.ty < 1) {
                                        // Algae is valid! do stuff!
                                }
                        }
                }
        });


        // Required for megatag2 in periodic() function before fetching pose.
        limelight.getSettings()
                .withRobotOrientation(new Orientation3d(m_gyro.getRotation3d(),
                        new AngularVelocity3d(m_gyro.getAngularVelocityXDevice().getValue(),
                        m_gyro.getAngularVelocityYDevice().getValue(),
                        m_gyro.getAngularVelocityZDevice().getValue())))
                .save();

        // Get MegaTag2 pose
        Optional<PoseEstimate> visionEstimate = limelight.getPoseEstimator(true).getPoseEstimate();
        // If the pose is present
        visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
                // Add it to the pose estimator.
                Pose2d pose = poseEstimate.pose.toPose2d();
                SignalLogger.writeDouble("Pose X: ", pose.getX());
                m_poseEstimator.addVisionMeasurement(pose, poseEstimate.timestampSeconds);
        });
    }
}
