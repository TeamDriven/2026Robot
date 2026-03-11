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
        final AutoTrajectory startPickupAndIntake = routine.trajectory("NeutralZoneDepot");
        routine.active().onTrue(
                startPickupAndIntake.resetOdometry()
                        .andThen(startPickupAndIntake.cmd()));
        return routine;
    }

}