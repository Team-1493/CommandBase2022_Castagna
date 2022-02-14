package frc.robot.commands.IntakeShooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeConveyor;
import frc.robot.subsystems.SwerveDriveSystem;

public class IntakeBallToLower extends SequentialCommandGroup {

      public IntakeBallToLower(IntakeConveyor intakeConveyor) {
        addCommands(
            new StartIntakeLowerConveyor(intakeConveyor),
            new stopIntakeLowerConveyor(intakeConveyor)
        );
    }
}




