// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BallTunnel;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static frc.robot.Subsystems.m_shooter;
import static frc.robot.Subsystems.m_intake;
import static frc.robot.Subsystems.m_ballTunnel;

import frc.robot.Constants.DrivetrainConst;
import frc.robot.commands.autos.OutpostAuto;

public class RobotContainer {
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final Telemetry logger = new Telemetry(DrivetrainConst.MaxSpeed);

	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

	private final AutoFactory autoFactory;
	private final OutpostAuto autoRoutines;
	private final AutoChooser autoChooser = new AutoChooser();


	public static SwerveModulePosition frontRight;

	public static SwerveModulePosition backRight;

	public static SwerveModulePosition frontLeft;

	public static SwerveModulePosition backLeft;

	public static final Translation2d m_frontLeftLocation = new Translation2d(TunerConstants.kFrontLeftXPos,
			TunerConstants.kFrontLeftYPos);

	public static final Translation2d m_frontRightLocation = new Translation2d(TunerConstants.kFrontRightXPos,
			TunerConstants.kFrontRightYPos);

	public static final Translation2d m_backLeftLocation = new Translation2d(TunerConstants.kBackLeftXPos,
			TunerConstants.kBackLeftYPos);

	public static final Translation2d m_backRightLocation = new Translation2d(TunerConstants.kBackRightXPos,
			TunerConstants.kBackRightYPos);

	public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
			m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

	public static Field2d m_field = new Field2d();

	public RobotContainer() {
		autoFactory = drivetrain.createAutoFactory();
		autoRoutines = new OutpostAuto(autoFactory);

		autoChooser.addRoutine("Outpost", autoRoutines::simplePathAuto);
		SmartDashboard.putData("Field", m_field);
		SmartDashboard.putData("Auto Chooser", autoChooser);
		configureBindings();

		frontLeft = drivetrain.getState().ModulePositions[0];
		frontRight = drivetrain.getState().ModulePositions[1];
		backLeft = drivetrain.getState().ModulePositions[2];
		backRight = drivetrain.getState().ModulePositions[3];
	}

	private void configureBindings() {
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivetrain.applyRequest(DriveControls.driveRequest()));

		// Idle while the robot is disabled. This ensures the configured
		// neutral mode is applied to the drive motors while disabled.
		final var idle = new SwerveRequest.Idle();
		RobotModeTriggers.disabled().whileTrue(
				drivetrain.applyRequest(() -> idle).ignoringDisable(true));

		// Controls.joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
		// Controls.joystick.b().whileTrue(drivetrain.applyRequest(
		// () -> point.withModuleDirection(new Rotation2d(-Controls.joystick.getLeftY(),
		// -Controls.joystick.getLeftX()))));

		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		Controlls.joystick.back().and(Controlls.joystick.y())
				.whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		Controlls.joystick.back().and(Controlls.joystick.x())
				.whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		Controlls.joystick.start().and(Controlls.joystick.y())
				.whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		Controlls.joystick.start().and(Controlls.joystick.x())
				.whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		// reset the field-centric heading on left bumper press
		Controlls.joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

		Controlls.joystick.b().whileTrue(m_ballTunnel.runBallTunnelCommand(28,100));


		drivetrain.registerTelemetry(logger::telemeterize);

		Controlls.joystick.a().onTrue(drivetrain.applyRequest(DriveControls.autoHeading()))
				.onFalse(drivetrain.applyRequest(DriveControls.driveRequest()));

		Controlls.joystick.rightTrigger().whileTrue(m_shooter.runShooterCommand(56.7, 100))
				.onFalse(new InstantCommand(() -> m_shooter.stopMotors()));
		 Controlls.joystick.rightBumper().whileTrue(m_intake.runIntakePercent(-0.5)).onFalse(m_intake.stopIntakeCommand());

		// new Trigger(m_intake.isSensorTripped()).onTrue(m_intake.feedCommand(50, 100)).onFalse(m_intake.stopIntakeCommand());

	}

	public Command getAutonomousCommand() {
		return autoChooser.selectedCommand();
	}
}
