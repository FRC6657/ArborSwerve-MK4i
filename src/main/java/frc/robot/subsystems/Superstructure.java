package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivebase.Drivebase;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  // Subsystems
  Drivebase drivebase;

  public Superstructure(Drivebase drivebase) {
    this.drivebase = drivebase;
  }

  public Command logMessage(String message) {
    return Commands.runOnce(() -> Logger.recordOutput("Command Log", message));
  }
}
