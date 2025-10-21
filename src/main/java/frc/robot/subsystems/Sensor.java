package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensor extends SubsystemBase {
  private DigitalInput m_intake_sensor;

  public Sensor() {
    m_intake_sensor = new DigitalInput(0);

    Shuffleboard.getTab("MAIN").addBoolean("Button", () -> getSensor());
  }

  public boolean getSensor() {
    return !m_intake_sensor.get();
  }

  @Override
  public void periodic() {}
}
