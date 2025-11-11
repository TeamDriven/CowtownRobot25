// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.RobotContainer.m_arm;
import static frc.robot.RobotContainer.m_claw;
import static frc.robot.RobotContainer.m_elevator;
import static frc.robot.RobotContainer.m_intake;
import static frc.robot.RobotContainer.m_pivot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoHome extends SequentialCommandGroup {
  /** Creates a new GoHome. */
  public GoHome() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(m_intake, m_arm, m_claw, m_elevator);
    addCommands(
        m_arm.setPosition(-15),
        new WaitUntilCommand(() -> m_arm.isAtSetpoint()),
        new WaitCommand(.5),
        m_elevator.setPosition(Elevator.level4Position),
        new WaitUntilCommand(() -> m_elevator.isAtSetpoint()),
        new WaitCommand(1),
        m_pivot.setPosition(-75),
        new WaitUntilCommand(() -> m_pivot.isAtSetpoint()),
        new WaitCommand(.5),
        m_arm.setPosition(Arm.home),
        new WaitUntilCommand(() -> m_arm.isAtSetpoint()),
        new WaitCommand(.5),
        m_elevator.setPosition(0),
        new WaitUntilCommand(() -> m_elevator.isAtSetpoint()),
        new WaitCommand(1),
        m_pivot.setPosition(0),
        new WaitUntilCommand(() -> m_pivot.isAtSetpoint()));
  }
}
