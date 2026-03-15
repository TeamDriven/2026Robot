package frc.robot.commands.autos;

import static frc.robot.Subsystems.m_angleController;
import static frc.robot.Subsystems.m_ballTunnel;
import static frc.robot.Subsystems.m_intakeRollers;
import static frc.robot.Subsystems.m_shooter;

import java.time.Instant;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootCommand;

public class OutpostNeutralAuto {
    private final AutoFactory m_factory;

    public OutpostNeutralAuto(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Outpost Netural");
        final AutoTrajectory movement1 = routine.trajectory("OutpostNetural", 0);
        final AutoTrajectory pickup = routine.trajectory("OutpostNetural",1);
        final AutoTrajectory shooting = routine.trajectory("OutpostNetural",2);
        final AutoTrajectory outpost = routine.trajectory("OutpostNetural",3);
        // final AutoTrajectory outpostPickup = routine.trajectory("OutpostNetural",4);

        routine.active().onTrue(
                movement1.resetOdometry()
                        .andThen(movement1.cmd()));

        movement1.done().onTrue(Commands.parallel(
            new InstantCommand(()-> m_intakeRollers.feedMotor(90,100)),
            pickup.cmd()
        ));

        pickup.done().onTrue(Commands.parallel(
            new InstantCommand(() -> m_shooter.runShooter(30, m_angleController.calculateHoodAngle())),
            shooting.cmd()
        ));

         shooting.done().onTrue(Commands.sequence(
                new InstantCommand(() -> RobotContainer.drivetrain.applyRequest(Controls.localHeading(Constants.FieldConst.RED_HUB))),
                new InstantCommand(() -> m_ballTunnel.runBallTunnel(62.5, 100)),
                new InstantCommand(() -> new WaitCommand(3)),
                new InstantCommand(() -> m_ballTunnel.stopBallTunnel()),
                new InstantCommand(() -> m_shooter.stopMotors()),
                outpost.cmd()
        ));

        





        return routine;
    }
}