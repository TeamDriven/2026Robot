package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConst;
import frc.robot.subsystems.LimelightSubsystem;

public class Controls {
    public static final CommandXboxController joystick = new CommandXboxController(0);

    public static final boolean isRightStickDrive = true;
    public static boolean autoAlign = false;

    public static Command autoAlignCommand() {
        return new Command() {
            @Override
            public void initialize() {
                autoAlign = !autoAlign;
                System.out.println("autoAlign: " + autoAlign);
            }
        };
    }

    private final static SwerveRequest.FieldCentric driveF = new SwerveRequest.FieldCentric()
                .withDeadband(DrivetrainConst.MaxSpeed * 0.1).withRotationalDeadband(DrivetrainConst.MaxAngularRate * 0.1) // Add
                                                                                                                           // a
                                                                                                                           // 10%
                                                                                                                           // deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final static SwerveRequest.RobotCentric driveR = new SwerveRequest.RobotCentric()
                        .withDeadband(DrivetrainConst.MaxSpeed * 0.1).withRotationalDeadband(DrivetrainConst.MaxAngularRate * 0.1) // Add
                                                                                                                                   // a
                                                                                                                                   // 10%
                                                                                                                                   // deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
            
                public static Supplier<SwerveRequest> driveRequest() {
                    if (autoAlign) {
                        final var rot_limelight = LimelightSubsystem.limelight_aim_proportional();
                        final var forward_limelight = LimelightSubsystem.limelight_range_proportional();
            
                        return () -> driveR
                        .withVelocityX(forward_limelight * DrivetrainConst.MaxSpeed)
                        .withVelocityY(
                                (isRightStickDrive ? joystick.getRightX() : joystick.getLeftX()) * DrivetrainConst.MaxSpeed)
                        .withRotationalRate(-rot_limelight * DrivetrainConst.MaxAngularRate);
    
            } else {
                return () -> driveF
                    .withVelocityX(
                            (isRightStickDrive ? joystick.getRightY() : joystick.getLeftY()) * DrivetrainConst.MaxSpeed)
                    .withVelocityY(
                            (isRightStickDrive ? joystick.getRightX() : joystick.getLeftX()) * DrivetrainConst.MaxSpeed)
                    .withRotationalRate(-(isRightStickDrive ? joystick.getLeftX() : joystick.getRightX())
                            * DrivetrainConst.MaxAngularRate);
        }
    }

}
