package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.DrivetrainConst;

import frc.robot.Constants.VisionConsts;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Subsystems.m_limelight;

public class Controls {
        public static final CommandXboxController joystick = new CommandXboxController(0);

        public static final Trigger climberUp = joystick.pov(90);
        public static final Trigger climberDown = joystick.pov(270);
        public static final Trigger intake = joystick.leftBumper();
        public static final Trigger cancel = joystick.leftStick();
        public static final Trigger resetHeading = joystick.start();
        public static final Trigger shoot = joystick.rightBumper();
        public static final Trigger autoLineUpOn = joystick.a();
        public static final Trigger autoLineUpOff = joystick.b();

        public static final boolean isRightStickDrive = false;

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
                        DoubleSupplier errorX = () -> (targetPose.getX()
                                        - RobotContainer.drivetrain.getState().Pose.getX());
                        // System.out.println(errorX.getAsDouble());

                        if (Math.abs(errorX.getAsDouble()) < 0.03) { // 3 cm
                                return 0.0;
                        }

                        // 1.5 is how fast to move to position
                        return MathUtil.clamp(
                                        errorX.getAsDouble() * 1.5,
                                        -DrivetrainConst.MaxSpeed,
                                        DrivetrainConst.MaxSpeed);
                };

                final DoubleSupplier ySpeed = () -> {
                        DoubleSupplier errorY = () -> (targetPose.getY()
                                        - RobotContainer.drivetrain.getState().Pose.getY());

                        if (Math.abs(errorY.getAsDouble()) < 0.03) { // 3 cm
                                return 0.0;
                        }
                        // 1.5 is how fast to move to position
                        return MathUtil.clamp(
                                        errorY.getAsDouble() * 1.5,
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

                        if (Math.abs(error) < 0.03) {
                                return 0.0;
                        }

                        return error * DrivetrainConst.MaxAngularRate; // simple P controller
                };

                return () -> driveF
                                .withVelocityX(-xSpeed.getAsDouble())
                                .withVelocityY(-ySpeed.getAsDouble())
                                .withRotationalRate(rotationRate.getAsDouble());
                // .withRotationalRate(0);
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
                                .withRotationalRate(0);
        }

}