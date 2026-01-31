package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.DrivetrainConst;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Subsystems.m_limelight;

public class Controlls {
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

        public static Supplier<SwerveRequest> autoHeading() {
                final var rot_limelight = m_limelight.limelight_aim_proportional();
                final var forward_limelight = m_limelight.limelight_range_proportional();

                return () -> driveR
                                .withVelocityX(-m_limelight.limelight_range_proportional())
                                .withVelocityY(
                                                -(isRightStickDrive ? joystick.getRightX() : joystick.getLeftX())
                                                                * DrivetrainConst.MaxSpeed)
                                .withRotationalRate(m_limelight.limelight_aim_proportional()
                                                * DrivetrainConst.MaxAngularRate / 10);
        }
}