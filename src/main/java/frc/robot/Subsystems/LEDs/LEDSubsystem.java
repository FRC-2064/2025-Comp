package frc.robot.Subsystems.LEDs;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private CANdle candle = new CANdle(LEDConstants.CANDLE_ID);
  
  public LEDSubsystem() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.GRB;
    candle.configAllSettings(config);
    candle.clearAnimation(0);
    off();
  }

  public void climbed() {
    candle.animate(new RainbowAnimation(1, 1, LEDConstants.NUM_LEDS));
  }

  public void off() {
    candle.clearAnimation(0);
    candle.setLEDs(0, 0, 0);
  }
}