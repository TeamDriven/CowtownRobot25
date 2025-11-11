package frc.robot.subsystems;

import static frc.robot.RobotContainer.m_pivot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;

public class Pivot extends SubsystemBase {
  private TalonFX pivotMotor = new TalonFX(14, "DriveBus");

  public static final double outPosition = -136;
  public static final double inPosition = 6; // 1.179688

  public static final double levelOnePosition = -50;
  public static final double outTheWay = -65;

  public static final double levelTwoPosition = -28.658202;
  // -4.776367 rotations	for l2

  MotionMagicVoltage motionMagicControls;

  VoltageOut voltageControl;
  NeutralOut stopMode;

  public Pivot() {
    initPivot();
    motionMagicControls = new MotionMagicVoltage(0);
    voltageControl = new VoltageOut(0);
    stopMode = new NeutralOut();
  }

  public void initPivot() {
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

    configs.MotionMagic.MotionMagicAcceleration =
        80; // target acceleration of 160 rps/s (0.5 seconds)
    configs.MotionMagic.MotionMagicCruiseVelocity = 40; // target curise velocity of 40 rps;
    configs.MotionMagic.MotionMagicJerk = 800;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = pivotMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK())
      System.out.println(
          "Could not apply configs to pivot motor, error code: " + status.toString());

    pivotMotor.setPosition(0);
  }

  public Command setPos(double position) {
    Command cmd = new InstantCommand(() -> runMotorToPosition(position));
    cmd.addRequirements(m_pivot);
    return cmd;
  }

  public Command setPosition(double position) {
    return new Command() {

      @Override
      public void execute() {
        runMotorToPosition(position);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  public Command setPosition(DoubleSupplier position) {

    return new Command() {
      @Override
      public void execute() {

        runMotorToPosition(position.getAsDouble());
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
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

  public Command resetPositionZero() {
    return new Command() {
      @Override
      public void execute() {
        pivotMotor.setPosition(0);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  public void runVoltage(double volts) {
    pivotMotor.setVoltage(volts);
  }

  public void stop() {
    pivotMotor.setControl(stopMode);
  }

  private void runMotorToPosition(double target) {
    pivotMotor.setControl(motionMagicControls.withPosition((target / 360) * 60));
  }

  //   public Command waitUntilAtPosition(double setPosition) {
  //     return new WaitUntilCommand(
  //         () -> {
  //           double currentPosition = pivotMotor.getPosition().getValueAsDouble() / 59.999999;
  //           System.out.println(currentPosition - setPosition);
  //           return Math.abs(currentPosition - setPosition) <= 10;
  //         });
  //   }

  public boolean isAtSetpoint() {
    return pivotMotor.getClosedLoopError().getValueAsDouble() / 60 < .25;
  }

  //   public double getCuttentPosition() {
  //     return pivotMotor.getPosition().getValueAsDouble() / 60;
  //   }

  public Command waitUntilAtPosition(DoubleSupplier setPosition) {
    return new WaitUntilCommand(
        () -> {
          double currentPosition = pivotMotor.getPosition().getValueAsDouble() / 59.999999;
          return Math.abs(currentPosition - setPosition.getAsDouble()) <= 10;
        });
  }

  @Override
  public void periodic() {}
}
