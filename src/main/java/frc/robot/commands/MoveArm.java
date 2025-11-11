package frc.robot.commands;

import static frc.robot.RobotContainer.m_arm;
import static frc.robot.RobotContainer.m_claw;
import static frc.robot.RobotContainer.m_elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class MoveArm extends SequentialCommandGroup {
  public MoveArm(double armPosition, double elevatorHeight) {
    addRequirements(m_arm, m_elevator, m_claw);
    addCommands(
        m_elevator.setPosition(elevatorHeight),
        new WaitCommand(1),
        m_arm.setPosition(armPosition),
        new WaitUntilCommand(() -> m_elevator.isAtSetpoint()),
        m_elevator.setPosition(elevatorHeight),
        new WaitUntilCommand(() -> m_elevator.isAtSetpoint()));
  }
}
