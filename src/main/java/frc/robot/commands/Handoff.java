package frc.robot.commands;

import static frc.robot.RobotContainer.m_arm;
import static frc.robot.RobotContainer.m_claw;
import static frc.robot.RobotContainer.m_elevator;
import static frc.robot.RobotContainer.m_intake;
import static frc.robot.RobotContainer.m_pivot;
import static frc.robot.RobotContainer.m_sensor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class Handoff extends SequentialCommandGroup {
  public Handoff() {
    addRequirements(m_intake, m_arm, m_claw, m_elevator, m_sensor);
    addCommands(
        m_pivot.setPosition(Pivot.levelOnePosition),
        m_elevator.setPosition(Elevator.level4Position),
        m_arm.setPosition(Arm.pickUpPos),
        new WaitUntilCommand(() -> m_arm.isAtSetpoint()),
        m_pivot.setPosition(Pivot.inPosition),
        m_elevator.setPosition(Elevator.intakePickUpPos),
        new WaitUntilCommand(() -> m_elevator.isAtSetpoint()),
        m_claw.runVoltageCommand(5),
        m_intake.runVoltageCommand(-5),
        new WaitUntilCommand(() -> !m_sensor.getSensor()),
        m_claw.stopMotor());
  }
}
