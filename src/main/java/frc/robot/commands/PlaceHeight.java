package frc.robot.commands;

import static frc.robot.RobotContainer.m_arm;
import static frc.robot.RobotContainer.m_claw;
import static frc.robot.RobotContainer.m_elevator;
import static frc.robot.RobotContainer.m_intake;
import static frc.robot.RobotContainer.m_pivot;
import static frc.robot.RobotContainer.m_sensor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class PlaceHeight extends SequentialCommandGroup {
  public PlaceHeight(double elevatorHieght, double armHeight, double pivotDegree) {
    addRequirements(m_elevator, m_arm, m_pivot, m_intake, m_sensor, m_claw);
    addCommands(
        m_elevator.setPosition(elevatorHieght),
        new WaitUntilCommand(() -> m_elevator.isAtSetpoint()),
        new WaitCommand(1),
        m_pivot.setPosition(pivotDegree),
        new WaitUntilCommand(() -> m_pivot.isAtSetpoint()),
        new WaitCommand(.5),

        // m_pivot.setPosition(-70),
        // new WaitUntilCommand(() -> m_pivot.isAtSetpoint()),
        // new WaitCommand(.75),

        m_arm.setPosition(armHeight),
        new WaitUntilCommand(() -> m_arm.isAtSetpoint()),
        new WaitUntilCommand(() -> m_elevator.isAtSetpoint()));
  }
}
