package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OutpostAuto {
    private final AutoFactory m_factory;

    public OutpostAuto(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Outpost Auto");
        final AutoTrajectory pickup1 = routine.trajectory("Outpost", 0);
        final AutoTrajectory shoot1 = routine.trajectory("Outpost", 1);
        final AutoTrajectory climb = routine.trajectory("Outpost", 2);

        routine.active().onTrue(
                pickup1.resetOdometry()
                        .andThen(pickup1.cmd()));

        pickup1.done().onTrue(Commands.sequence(
                new WaitCommand(1),
                shoot1.cmd()));

        shoot1.done().onTrue(Commands.sequence(
                new WaitCommand(1),
                climb.cmd()));

        return routine;
    }
}