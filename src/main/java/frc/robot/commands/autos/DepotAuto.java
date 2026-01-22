package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.MatchStartCommand;

public class DepotAuto {
    private final AutoFactory m_factory;

    public DepotAuto(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Depot Climb Auto");
        final AutoTrajectory pickup1 = routine.trajectory("Depot_Climb", 0);
        final AutoTrajectory shoot1 = routine.trajectory("Depot_Climb", 1);
        final AutoTrajectory climb = routine.trajectory("Depot_Climb", 2);

        routine.active().onTrue(Commands.sequence(
                pickup1.resetOdometry(),
                Commands.parallel(
                    pickup1.cmd(),
                    new MatchStartCommand()
                )
        ));

        pickup1.done().onTrue(Commands.sequence(
                new WaitCommand(1),
                shoot1.cmd()));

        shoot1.done().onTrue(Commands.sequence(
                new WaitCommand(1),
                climb.cmd()));
                
        return routine;
    }
}