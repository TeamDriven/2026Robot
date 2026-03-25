package frc.robot.commands.autos;

import static frc.robot.Subsystems.m_angleController;
import static frc.robot.Subsystems.m_ballTunnel;
import static frc.robot.Subsystems.m_intakeActuation;
import static frc.robot.Subsystems.m_intakeRollers;
import static frc.robot.Subsystems.m_shooter;

import java.time.Instant;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.IntakeActuation;

public class OutpostNeutralAuto {
    private final AutoFactory m_factory;

    public OutpostNeutralAuto(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Outpost Netural");
        final AutoTrajectory movement1 = routine.trajectory("OutpostNetural", 0);
        final AutoTrajectory pickup = routine.trajectory("OutpostNetural", 1);
        final AutoTrajectory shooting = routine.trajectory("OutpostNetural", 2);
        final AutoTrajectory outpost = routine.trajectory("OutpostNetural", 3);
        final AutoTrajectory outpostShooting = routine.trajectory("OutpostNetural", 4);

        routine.active().onTrue(
                m_intakeActuation.setPositionUntilSupply(1.39).andThen(
                        movement1.resetOdometry())
                        .andThen(movement1.cmd()));

        movement1.done().onTrue(Commands.parallel(
                new InstantCommand(() -> m_intakeRollers.feedMotor(90, 100)),
                pickup.cmd()));

        pickup.done().onTrue(Commands.parallel(
                new InstantCommand(() -> m_shooter.runShooter(30, m_angleController.calculateHoodAngle())),
                shooting.cmd()));

        shooting.done().onTrue(Commands.sequence(
                new InstantCommand(() -> m_angleController.setPosition(12)),
                new InstantCommand(() -> RobotContainer.drivetrain.applyRequest(
                        Controls.localHeading(Robot.alliance == Alliance.Red ? Constants.FieldConst.RED_HUB
                                : Constants.FieldConst.BLUE_HUB))),
                new InstantCommand(() -> m_ballTunnel.runBallTunnel(62.5, 100)).alongWith(
                        new InstantCommand(() -> m_intakeRollers.feedMotor(85, 100)), new InstantCommand(() -> m_ballTunnel.runHopper(62.5, 100))),
                m_intakeActuation.intakeInSlowCommand(0.3),
                new WaitCommand(4),
                new InstantCommand(() -> m_angleController.setPosition(0)),
                new InstantCommand(() -> m_ballTunnel.stopBallTunnel()),
                new InstantCommand(() -> m_shooter.stopMotors()),
                m_intakeActuation.setPositionUntilSupply(1.39),
                outpost.cmd()

        ));

        outpost.done().onTrue(Commands.sequence(
                new WaitCommand(1),
                new InstantCommand(() -> m_shooter.runShooter(23, m_angleController.calculateHoodAngle())),
                outpostShooting.cmd()

        ));

        outpostShooting.done().onTrue(Commands.sequence(
                new InstantCommand(() -> m_angleController.setPosition(12)),
                new InstantCommand(() -> RobotContainer.drivetrain.applyRequest(
                        Controls.localHeading(Robot.alliance == Alliance.Red ? Constants.FieldConst.RED_HUB
                                : Constants.FieldConst.BLUE_HUB))),
                new InstantCommand(() -> m_ballTunnel.runBallTunnel(62.5, 100)).alongWith(
                        m_intakeRollers.feedCommand(85, 100),
                        m_intakeActuation.intakeInSlowCommand(0.5)),
                new WaitCommand(4),

                new InstantCommand(() -> m_angleController.setPosition(0)),
                new InstantCommand(() -> m_ballTunnel.stopBallTunnel()),
                new InstantCommand(() -> m_shooter.stopMotors()),
                m_intakeActuation.setPositionUntilSupply(1.39)

        ));

        return routine;
    }
}