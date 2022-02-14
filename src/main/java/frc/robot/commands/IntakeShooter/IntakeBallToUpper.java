package frc.robot.commands.IntakeShooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeConveyor;

public class IntakeBallToUpper extends SequentialCommandGroup {

      public IntakeBallToUpper(IntakeConveyor intakeConveyor) {
        addCommands(
            new ParallelCommandGroup(
                new startUpperConveyor(intakeConveyor),
                new StartIntakeLowerConveyor(intakeConveyor)),
            new stopUpperConveyor(intakeConveyor)
        );
    }
}