package frc.robot.commands;

import static frc.robot.RobotContainer.m_intake;
import static frc.robot.RobotContainer.m_pivot;
import static frc.robot.RobotContainer.m_sensor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Pivot;

public class PickUpCoral extends SequentialCommandGroup {

  public PickUpCoral() {
    addRequirements(m_intake, m_pivot, m_sensor);
    addCommands(
        m_pivot.setPosition(Pivot.outPosition),
        m_intake.runVoltageCommand(7),
        new WaitUntilCommand(() -> m_sensor.getSensor()),
        m_pivot.setPosition(0),
        new WaitUntilCommand(() -> m_pivot.isAtSetpoint()),
        new WaitCommand(0.5),
        m_intake.runVoltageCommand(5),
        new WaitCommand(.75),
        m_intake.runVoltageCommand(0));
  }
}
