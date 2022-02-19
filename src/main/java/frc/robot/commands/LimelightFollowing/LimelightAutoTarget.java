package frc.robot.commands.LimelightFollowing;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDriveSystem;

public class LimelightAutoTarget extends SequentialCommandGroup {

      public LimelightAutoTarget(SwerveDriveSystem m_sds, Supplier<double[]> m_stickState) {
        addCommands(
            new LimelightSearch(m_sds, m_stickState),
            new LimelightAlign(m_sds)
        );
    }
}




