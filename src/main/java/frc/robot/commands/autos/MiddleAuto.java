package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.MatchStartCommand;

public class MiddleAuto {
    private final AutoFactory m_factory;

    public MiddleAuto(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Middle Climb Auto");
        final AutoTrajectory climb = routine.trajectory("Middle_Climb");

        routine.active().onTrue(Commands.sequence(
                climb.resetOdometry(),
                Commands.parallel(
                    climb.cmd(),
                    new MatchStartCommand()
                )));
                
        return routine;
    }
}