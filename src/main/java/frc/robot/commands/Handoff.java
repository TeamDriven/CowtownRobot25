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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class Handoff extends SequentialCommandGroup {
  public Handoff() {
    addRequirements(m_intake, m_arm, m_claw, m_elevator, m_sensor, m_pivot);
    addCommands(
        m_pivot.setPosition(-75),
        new WaitUntilCommand(() -> m_pivot.isAtSetpoint()),
        new WaitCommand(.5),
        m_elevator.setPosition(-6),
        m_arm.setPosition(Arm.pickUpPos),
        new WaitUntilCommand(() -> m_arm.isAtSetpoint()),
        new WaitCommand(.5),
        m_elevator.setPosition(-6),
        new WaitUntilCommand(() -> m_elevator.isAtSetpoint()),
        new WaitCommand(1),
        m_pivot.setPosition(Pivot.inPosition),
        new WaitUntilCommand(() -> m_pivot.isAtSetpoint()),
        new WaitCommand(1),
        m_elevator.setPosition(Elevator.intakePickUpPos), // -31.280762
        new WaitCommand(1),
        new WaitUntilCommand(() -> m_elevator.isAtSetpoint()),
        new WaitUntilCommand(() -> m_arm.isAtSetpoint()),
        m_claw.runVoltageCommand(-4), // 1.179688
        m_intake.runVoltageCommand(-10),
        new WaitUntilCommand(() -> !m_sensor.getSensor()),
        new WaitCommand(1),
        m_intake.stopMotor(),
        m_claw.runVoltageCommand(0));
  }
}
