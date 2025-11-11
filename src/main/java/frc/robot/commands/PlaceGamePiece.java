package frc.robot.commands;

import static frc.robot.RobotContainer.m_arm;
import static frc.robot.RobotContainer.m_claw;
import static frc.robot.RobotContainer.m_elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class PlaceGamePiece extends SequentialCommandGroup {
  public PlaceGamePiece(double armAngle) {
    addRequirements(m_claw, m_elevator, m_arm);
    addCommands(
        m_arm.setPosition(armAngle),
        new WaitUntilCommand(() -> m_arm.isAtSetpoint()),
        new WaitCommand(1),
        m_claw.runVoltageCommand(5),
        // new WaitUntilCommand(() -> (!m_claw.hasGamePiece())),
        new WaitCommand(.75),
        m_claw.stopMotor());
    // m_arm.setPosition(Arm.pickUpPos),
    // new WaitUntilCommand(() -> m_arm.isAtSetpoint()),
    // m_elevator.setPosition(Elevator.restPos),
    // new WaitUntilCommand(() -> m_elevator.isAtSetpoint()),
    // new WaitCommand(.5),
    // m_elevator.stopMotor(),
    // m_arm.stopMotor());
  }
}
