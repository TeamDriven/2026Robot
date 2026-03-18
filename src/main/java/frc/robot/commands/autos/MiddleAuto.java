package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootCommand;

import frc.robot.RobotContainer;
import static frc.robot.Subsystems.m_angleController;
import static frc.robot.Subsystems.m_ballTunnel;
import static frc.robot.Subsystems.m_intakeActuation;
import static frc.robot.Subsystems.m_intakeRollers;
import static frc.robot.Subsystems.m_shooter;

public class MiddleAuto {
    private final AutoFactory m_factory;

    public MiddleAuto(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine  middleAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Middle Auto");
        final AutoTrajectory moveBack = routine.trajectory("Middle");
        routine.active().onTrue(
                moveBack.resetOdometry()
                        .andThen(moveBack.cmd()));


        moveBack.done().onTrue(Commands.sequence(
             new ShootCommand(23, m_angleController.calculateHoodAngle(), 62.5),
             new WaitCommand(8), 
             new InstantCommand(() -> m_angleController.setPosition(0)),
             new InstantCommand(() -> m_shooter.stopMotors()),
             new InstantCommand(() -> m_ballTunnel.stopBallTunnel())));
        return routine;
    }
}
