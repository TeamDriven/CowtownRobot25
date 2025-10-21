package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {

  private TalonFX leftMotor = new TalonFX(13, "DriveBus");

  public static final double l2Pos = -129;
  public static final double l4Pos = -150;
  public static final double pickUpPos = -176.989091; // needs to be -30.42

  private final double forwardSoftLimit = 90; // max angle in degrees
  private final double reverseSoftLimit = -90; // min angle in degrees
  //   public static final double de
  //     public static final double
  MotionMagicVoltage motionMagicControls;

  VoltageOut voltageControl;
  NeutralOut stopMode;

  public Arm() {
    initArm();
    motionMagicControls = new MotionMagicVoltage(0);
    voltageControl = new VoltageOut(0);
    stopMode = new NeutralOut();
  }

  public void initArm() {
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

    // SoftwareLimitSwitchConfigs softLimits = configs.SoftwareLimitSwitch;
    // softLimits.ForwardSoftLimitThreshold = forwardSoftLimit * Math.PI / 180;
    // softLimits.ForwardSoftLimitEnable = true;
    // softLimits.ReverseSoftLimitThreshold = reverseSoftLimit * Math.PI / 180;
    // softLimits.ReverseSoftLimitEnable = true;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = leftMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK())
      System.out.println("Could not apply configs to left motor, error code: " + status.toString());

    leftMotor.setPosition(0);
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

  public void runVoltage(double volts) {
    leftMotor.setVoltage(volts);
  }

  public void stop() {
    leftMotor.setControl(stopMode);
  }

  private void runMotorToPosition(double target) { // 5.55.
    leftMotor.setControl(motionMagicControls.withPosition(target * 61.875 / 360));
  }

  public boolean isAtSetpoint() {
    return leftMotor.getClosedLoopError().getValueAsDouble() / 61.875 < .25;
  }

  public Command resetPositionZero() {
    return new Command() {
      @Override
      public void execute() {
        leftMotor.setPosition(0);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  //   public Command waitUntilAtPosition(double setPosition) {
  //     return new WaitUntilCommand(
  //         () -> {
  //           double currentPosition = leftMotor.getPosition().getValueAsDouble() / 59.999999;
  //           System.out.println(currentPosition - setPosition);
  //           return Math.abs(currentPosition - setPosition) <= 10;
  //         });
  //   }

  //   public Command waitUntilAtPosition(DoubleSupplier setPosition) {
  //     return new WaitUntilCommand(
  //         () -> {
  //           double currentPosition = leftMotor.getPosition().getValueAsDouble() / 59.999999;
  //           return Math.abs(currentPosition - setPosition.getAsDouble()) <= 10;
  //         });
  //   }

  @Override
  public void periodic() {}
}
