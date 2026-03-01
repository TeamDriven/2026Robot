package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class NeutralOutpostAuto {
    private final AutoFactory m_factory;

    public NeutralOutpostAuto(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine neutralZoneAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Neutral Zone Outpost");
        final AutoTrajectory startPickupAndIntake = routine.trajectory("NeutralZoneOutpost", 0);
        final AutoTrajectory startshooter = routine.trajectory("NeutralZoneOutpost", 1);
        final AutoTrajectory climb1 = routine.trajectory("NeutralZoneOutpost", 2);

        routine.active().onTrue(
                startPickupAndIntake.resetOdometry()
                        .andThen(startPickupAndIntake.cmd()));

        startPickupAndIntake.done().onTrue(Commands.sequence(
                new WaitCommand(1),
                // start intake code
                startshooter.cmd()));

        startshooter.done().onTrue(Commands.sequence(
                new WaitCommand(1),
                // start shooter
                climb1.cmd()));

        return routine;
    }

}