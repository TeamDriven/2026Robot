package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class NeutralDepotAuto {
    private final AutoFactory m_factory;

    public NeutralDepotAuto(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine neutralZoneAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Neutral Zone Depot");
        final AutoTrajectory startPickupAndIntake = routine.trajectory("NeutralZoneDepot", 0);
        final AutoTrajectory startshooter = routine.trajectory("NeutralZoneDepot", 1);
        final AutoTrajectory climb1 = routine.trajectory("NeutralZoneDepot", 2);

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