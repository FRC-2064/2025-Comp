package frc.robot.Subsystems.LEDs;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private CANdle candle = new CANdle(LEDConstants.CANDLE_ID);

  public LEDSubsystem() {
    candle.clearAnimation(0);
  }

  public void setCandleOn() {
    candle.setLEDs( 255, 255, 255);
  }

  public void setCandleOff() {
    candle.setLEDs( 0, 0, 0);
  }

  }