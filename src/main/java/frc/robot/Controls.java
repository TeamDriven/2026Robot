package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.DrivetrainConst;

import frc.robot.Constants.VisionConsts;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Subsystems.m_limelight;

public class Controls {
        public static final CommandXboxController joystick = new CommandXboxController(0);

        public static final Trigger climberUp = joystick.pov(90);
        public static final Trigger climberDown = joystick.pov(270);
        public static final Trigger intakeOut = joystick.x();
        public static final Trigger intakeIn = joystick.b();
        public static final Trigger cancel = joystick.y();
        public static final Trigger resetHeading = joystick.start();
        public static final Trigger shoot = joystick.rightBumper();
        public static final Trigger autoLineUpOn = joystick.a();
        public static final Trigger autoLineUpOff = joystick.b();

        // TODO To change drive to be on left stick make sure isRightStickDrive is false
        // MATYLDA!
        public static final boolean isRightStickDrive = true;

        private static final SwerveRequest.FieldCentric driveF = new SwerveRequest.FieldCentric()
                        .withDeadband(DrivetrainConst.MaxSpeed * 0.1)
                        .withRotationalDeadband(DrivetrainConst.MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private static final SwerveRequest.RobotCentric driveR = new SwerveRequest.RobotCentric()
                        .withDeadband(DrivetrainConst.MaxSpeed * 0.1)
                        .withRotationalDeadband(DrivetrainConst.MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        public static Supplier<SwerveRequest> driveRequest() {

                return () -> driveF
                                .withVelocityX(
                                                -(isRightStickDrive ? joystick.getRightY() : joystick.getLeftY())
                                                                * DrivetrainConst.MaxSpeed)
                                .withVelocityY(
                                                -(isRightStickDrive ? joystick.getRightX() : joystick.getLeftX())
                                                                * DrivetrainConst.MaxSpeed)
                                .withRotationalRate(
                                                -(isRightStickDrive ? joystick.getLeftX() : joystick.getRightX())
                                                                * DrivetrainConst.MaxAngularRate);
        }

        /*
         * Turn towards given Pose2d
         */
        public static Supplier<SwerveRequest> localHeading(Pose2d target) {

                // Get the target Angle
                final DoubleSupplier targetAngle = () -> {
                        double dx = target.getX()
                                        - RobotContainer.drivetrain.getState().Pose.getX();

                        double dy = target.getY()
                                        - RobotContainer.drivetrain.getState().Pose.getY();

                        return Math.atan2(dy, dx); // radians
                };

                // Get the rotation rate -1 to 1 needed to get to target angle
                final DoubleSupplier rotationRate = () -> {
                        double currentHeading = RobotContainer.drivetrain.getState().Pose.getRotation().getRadians();
                        double error = targetAngle.getAsDouble() - currentHeading;

                        // Wrap to [-pi, pi]
                        error = Math.atan2(Math.sin(error), Math.cos(error));

                        return error * DrivetrainConst.MaxAngularRate; // simple P controller
                };

                return () -> driveF
                                .withVelocityX(-(isRightStickDrive ? joystick.getRightY() : joystick.getLeftY())
                                                * DrivetrainConst.MaxSpeed)
                                .withVelocityY(
                                                -(isRightStickDrive ? joystick.getRightX() : joystick.getLeftX())
                                                                * DrivetrainConst.MaxSpeed)
                                .withRotationalRate(rotationRate.getAsDouble());
        }

        /*
         * Travel to given Pose2d
         */
        public static Supplier<SwerveRequest> goToPositionAndRotation(Pose2d targetPose, Pose2d targetPointTo) {
                final DoubleSupplier xSpeed = () -> {
                        double errorX = targetPose.getX()
                                        - RobotContainer.drivetrain.getState().Pose.getX();

                        return MathUtil.clamp(
                                        errorX * 0.5,
                                        -DrivetrainConst.MaxSpeed,
                                        DrivetrainConst.MaxSpeed);
                };

                final DoubleSupplier ySpeed = () -> {
                        double errorY = targetPose.getY()
                                        - RobotContainer.drivetrain.getState().Pose.getY();

                        return MathUtil.clamp(
                                        errorY * 0.5,
                                        -DrivetrainConst.MaxSpeed,
                                        DrivetrainConst.MaxSpeed);
                };

                // Get the target Angle
                final DoubleSupplier targetAngle = () -> {
                        double dx = targetPointTo.getX()
                                        - RobotContainer.drivetrain.getState().Pose.getX();

                        double dy = targetPointTo.getY()
                                        - RobotContainer.drivetrain.getState().Pose.getY();

                        return Math.atan2(dy, dx); // radians
                };

                // Get the rotation rate -1 to 1 needed to get to target angle
                final DoubleSupplier rotationRate = () -> {
                        double currentHeading = RobotContainer.drivetrain.getState().Pose.getRotation().getRadians();
                        double error = targetAngle.getAsDouble() - currentHeading;

                        // Wrap to [-pi, pi]
                        error = Math.atan2(Math.sin(error), Math.cos(error));

                        return error * DrivetrainConst.MaxAngularRate; // simple P controller
                };

                return () -> driveF
                                .withVelocityX(-xSpeed.getAsDouble())
                                .withVelocityY(-ySpeed.getAsDouble())
                                .withRotationalRate(rotationRate.getAsDouble());
        }

        /*
         * Travel to given Pose2d
         */
        public static Supplier<SwerveRequest> goToPosition(Pose2d target) {
                final DoubleSupplier xSpeed = () -> {
                        double errorX = target.getX()
                                        - RobotContainer.drivetrain.getState().Pose.getX();

                        return MathUtil.clamp(
                                        errorX * 0.5,
                                        -DrivetrainConst.MaxSpeed,
                                        DrivetrainConst.MaxSpeed);
                };

                final DoubleSupplier ySpeed = () -> {
                        double errorY = target.getY()
                                        - RobotContainer.drivetrain.getState().Pose.getY();

                        return MathUtil.clamp(
                                        errorY * 0.5,
                                        -DrivetrainConst.MaxSpeed,
                                        DrivetrainConst.MaxSpeed);
                };

                // Get the target Angle
                final DoubleSupplier targetAngle = () -> {
                        double dx = target.getX()
                                        - RobotContainer.drivetrain.getState().Pose.getX();

                        double dy = target.getY()
                                        - RobotContainer.drivetrain.getState().Pose.getY();

                        return Math.atan2(dy, dx); // radians
                };

                // Get the rotation rate -1 to 1 needed to get to target angle
                final DoubleSupplier rotationRate = () -> {
                        double currentHeading = RobotContainer.drivetrain.getState().Pose.getRotation().getRadians();
                        double error = targetAngle.getAsDouble() - currentHeading;

                        // Wrap to [-pi, pi]
                        error = Math.atan2(Math.sin(error), Math.cos(error));

                        return error * DrivetrainConst.MaxAngularRate; // simple P controller
                };

                return () -> driveF
                                .withVelocityX(-xSpeed.getAsDouble())
                                .withVelocityY(-ySpeed.getAsDouble())
                                .withRotationalRate(rotationRate.getAsDouble());
        }

}