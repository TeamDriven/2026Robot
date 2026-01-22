package frc.robot;

import java.lang.reflect.Parameter;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.DrivetrainConst;
import static frc.robot.Controlls.*;

import static frc.robot.Subsystems.m_limelight;

public class DriveControls {
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

        public static Supplier<SwerveRequest> autoHeading() {
                final var rot_limelight = m_limelight.limelight_aim_proportional();
                final var forward_limelight = m_limelight.limelight_range_proportional();
                final var finalDistance = 0.61;
                 

                // 1.11125m for apriltag #12
                double height = 1.11125;

                // tan(angle) = op/ad
                // ad = op/tan(angle)
               
               return () -> driveR
                                .withVelocityX((Math.abs(1.11125/Math.tan(LimelightHelpers.getTY("limelight"))) <= finalDistance) ? 0 : -m_limelight.limelight_range_proportional())
                                .withVelocityY(
                                                -(isRightStickDrive ? joystick.getRightX() : joystick.getLeftX())
                                                                * DrivetrainConst.MaxSpeed)
                                .withRotationalRate(m_limelight.limelight_aim_proportional()
                                                * DrivetrainConst.MaxAngularRate / 10);
                                                
        }
}
