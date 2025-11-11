package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

public class Elevator extends SubsystemBase {
  private TalonFX leftMotor = new TalonFX(15, "DriveBus");
  private TalonFX rightMotor = new TalonFX(16, "DriveBus");

  public static final double level2Position = -.689288; // -4.498047 rotations
  public static final double level3Position = -2.49564; // -16.285645 rotations
  public static final double level4Position = -6.5;
  public static final double intakePickUpPos = -5;
  public static final double restPos = -6;
  public static final double MAX_HEIGHT = -6.708;

  MotionMagicVoltage motionMagicControls;

  VoltageOut voltageControl;
  NeutralOut stopMode;

  public Elevator() {
    initElevator();
    motionMagicControls = new MotionMagicVoltage(0);
    voltageControl = new VoltageOut(0);
    stopMode = new NeutralOut();
  }

  public void initElevator() {
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
      status = leftMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK())
      System.out.println("Could not apply configs to left motor, error code: " + status.toString());

    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    for (int i = 0; i < 5; ++i) {
      status = rightMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK())
      System.out.println(
          "Could not apply configs to right motor, error code: " + status.toString());

    leftMotor.setPosition(0);
    rightMotor.setPosition(0);

    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
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
    // rightMotor.setVoltage(volts);
  }

  public void stop() {
    leftMotor.setControl(stopMode);
    // rightMotor.setControl(stopMode);
  }

  private void runMotorToPosition(double target) { // 5.55
    leftMotor.setControl(motionMagicControls.withPosition(target * (50.0 / 13) * (50.9 / 30)));
    // rightMotor.setControl(motionMagicControls.withPosition(target * (50.0 / 13) * (50.0 / 30)));
    // -31.717773
  }

  public boolean isAtSetpoint() {
    return leftMotor.getClosedLoopError().getValueAsDouble() / 59.999 < .25;
  }

  public Command resetPositionZero() {
    return new Command() {
      @Override
      public void execute() {
        leftMotor.setPosition(0);
        // rightMotor.setPosition(0);
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
