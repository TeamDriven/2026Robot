package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.DrivetrainConst;

import frc.robot.Constants.VisionConsts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Subsystems.m_limelight;

public class Controls {
        private static final CommandXboxController joystick = new CommandXboxController(0);

        public static final Trigger climberUp = joystick.pov(90);
        public static final Trigger climberDown = joystick.pov(270);
        public static final Trigger intakeOut = joystick.x();
        public static final Trigger intakeIn = joystick.b();
        public static final Trigger cancel = joystick.y();
        public static final Trigger resetHeading = joystick.start();
        public static final Trigger shoot = joystick.rightBumper();
        public static final Trigger autoLineUp = joystick.a();

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

        // public static Supplier<SwerveRequest> autoHeading() {
        // var rot_limelight = m_limelight.limelight_aim_proportional();
        // var forward_limelight = m_limelight.limelight_range_proportional();
        // var XVelocity = ((m_limelight.getAprilTagHeight() -
        // VisionConsts.LIMELIGHT_HEIGHT_1)
        // / (Math.tan(VisionConsts.LIMELIGHT_ANGLE + forward_limelight)))
        // - VisionConsts.DIST_TO_STOP;
        // SmartDashboard.putNumber("Limelight X", XVelocity);
        // return () -> driveR
        // .withVelocityX(XVelocity)
        // .withVelocityY(-(isRightStickDrive ? joystick.getRightX() :
        // joystick.getLeftX()) * DrivetrainConst.MaxSpeed)
        // .withRotationalRate(rot_limelight * DrivetrainConst.MaxAngularRate / 10);
        // }

        public static Supplier<SwerveRequest> autoHeading() {
                final var rot_limelight = m_limelight.limelight_aim_proportional();
                final var forward_limelight = m_limelight.limelight_range_proportional();
                var XDistance = ((m_limelight.getAprilTagHeight() - VisionConsts.LIMELIGHT_HEIGHT_1)
                                / (Math.tan(VisionConsts.LIMELIGHT_ANGLE + LimelightHelpers.getTY("limelight"))));
                // SmartDashboard.putNumber("Limelight X", XDistance);
                System.out.println("XDistance: " + XDistance);
                return () -> driveR
                                .withVelocityX(lineUpXSupplier(XDistance, m_limelight.limelight_range_proportional()))
                                .withVelocityY(
                                                -(isRightStickDrive ? joystick.getRightX() : joystick.getLeftX())
                                                                * DrivetrainConst.MaxSpeed)
                                .withRotationalRate(m_limelight.limelight_aim_proportional()
                                                * DrivetrainConst.MaxAngularRate / 10);
        }

        private static double lineUpXSupplier(double distance, double input) {
                if (distance - VisionConsts.DIST_TO_STOP <= 0) {
                        return 0;
                } else {
                        return -input;
                }
        }

        // public static Supplier<SwerveRequest> localHeading() {

        //         final DoubleSupplier targetAngle = () -> {
        //                 double dx = Constants.FieldConst.RED_HUB.getX()
        //                                 - Robot.m_poseEstimator.getEstimatedPosition().getX();

        //                 double dy = Constants.FieldConst.RED_HUB.getY()
        //                                 - Robot.m_poseEstimator.getEstimatedPosition().getY();

        //                 return Math.atan2(dy, -dx); // radians
        //         };

        //         final DoubleSupplier rotationRate = () -> {
        //                 double currentHeading = Robot.m_poseEstimator.getEstimatedPosition().getRotation().getRadians();
        //                 double error = targetAngle.getAsDouble() - currentHeading;

        //                 // Wrap to [-pi, pi]
        //                 error = Math.atan2(Math.sin(error), Math.cos(error));

        //                 return error * DrivetrainConst.MaxAngularRate; // simple P controller
        //         };

        //         return () -> driveF
        //                         .withVelocityX(-(isRightStickDrive ? joystick.getRightY() : joystick.getLeftY())
        //                                         * DrivetrainConst.MaxSpeed)
        //                         .withVelocityY(
        //                                         -(isRightStickDrive ? joystick.getRightX() : joystick.getLeftX())
        //                                                         * DrivetrainConst.MaxSpeed)
        //                         .withRotationalRate(rotationRate.getAsDouble());
        // }

}