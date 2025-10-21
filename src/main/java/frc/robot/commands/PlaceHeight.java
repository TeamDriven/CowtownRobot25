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

public class PlaceHeight extends SequentialCommandGroup {
  public PlaceHeight(double elevatorHieght, double armHeight) {
    addRequirements(m_elevator, m_arm, m_pivot, m_intake, m_sensor, m_claw);
    addCommands(
        m_elevator.setPosition(Elevator.level4Position),
        new WaitCommand(3),
        m_elevator.stopMotor(),
        m_arm.setPosition(-45),
        new WaitUntilCommand(() -> m_elevator.isAtSetpoint()),
        m_claw.runVoltageCommand(-5),
        new WaitUntilCommand(() -> (!m_claw.hasGamePiece())),
        m_claw.stopMotor(),
        m_elevator.setPosition(Elevator.restPos),
        m_arm.setPosition(Arm.pickUpPos),
        m_elevator.stopMotor(),
        m_arm.stopMotor());
  }
}
