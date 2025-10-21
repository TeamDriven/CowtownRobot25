package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private TalonFX clawMotor = new TalonFX(17, "DriveBus");

  VoltageOut voltageControl;
  NeutralOut stopMode;

  public Claw() {
    initClaw();
    voltageControl = new VoltageOut(0);
    stopMode = new NeutralOut();
  }

  public void initClaw() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 25;

    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot0.kP = 25;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.3;
    configs.Slot0.kV = 0.12;

    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = clawMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK())
      System.out.println(
          "Could not apply configs to intake motor, error code: " + status.toString());
  }

  public Command stopMotor() {
    return new Command() {
      @Override
      public void execute() {
        stop();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  public Command runVoltageCommand(double volts) {
    return new Command() {
      @Override
      public void execute() {
        runVoltage(volts);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  private void runVoltage(double volts) {
    clawMotor.setVoltage(volts);
  }

  public void stop() {
    clawMotor.setControl(stopMode);
  }

  public double getCurrents() {
    return clawMotor.getStatorCurrent().getValueAsDouble();
    // return clawMotor.getCurr
  }

  public boolean hasGamePiece() {
    return getCurrents() >= 20;
  }

  @Override
  public void periodic() {}
}
