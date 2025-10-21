package frc.robot.commands;

import static frc.robot.RobotContainer.m_arm;
import static frc.robot.RobotContainer.m_claw;
import static frc.robot.RobotContainer.m_elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;

public class MoveArm extends SequentialCommandGroup {
  public MoveArm(double armHeight, double elevatorHeight) {
    addRequirements(m_arm, m_elevator, m_claw);
    addCommands(
        m_elevator.setPosition(Elevator.restPos),
        new WaitCommand(3),
        m_elevator.stopMotor(),
        m_arm.setPosition(0),
        new WaitUntilCommand(() -> m_elevator.isAtSetpoint()),
        m_claw.stopMotor());
  }
}
