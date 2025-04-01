package frc.robot.Subsystems.LEDs;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.LEDConstants;
import frc.robot.Utils.Enums.LEDState;

public class LEDSubsystem extends SubsystemBase {
  private CANdle candle = new CANdle(LEDConstants.CANDLE_ID);

  private Animation currentAnimation = null;
  private int animChannel = 0;

  private LEDState state = LEDState.OFF;

  public LEDSubsystem() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.GRB;
    candle.configAllSettings(config);
    setAnimationForState(state);
  }

  public void setState(LEDState newState) {
    if (newState != state) {
      state = newState;
      setAnimationForState(state);
    }
  }

  private void setAnimationForState(LEDState state) {
    switch (state) {
      case OFF:
        candle.clearAnimation(animChannel);
        candle.setLEDs(0, 0, 0);
        currentAnimation = null;
        break;
      case LIGHTBAR:
       candle.setLEDs(255,255,255);
        currentAnimation = null;
        break;
      case HAS_PIECE:
        animChannel = 1;
        currentAnimation = new FireAnimation(0.5, 0.7, LEDConstants.NUM_LEDS, 0.8, 0.5, false, 0);
        break;
      case NO_PIECE:
        animChannel = 2;
        currentAnimation = new TwinkleAnimation(30, 70, LEDConstants.NUM_LEDS, 0, 0.4, LEDConstants.NUM_LEDS, TwinkleAnimation.TwinklePercent.Percent42, 0);
        break;
      case CLIMBED:
        animChannel = 3;
        currentAnimation = new RainbowAnimation(1, 0.7, LEDConstants.NUM_LEDS, false, 0);
        break;
      default:
        candle.clearAnimation(animChannel);
        candle.setLEDs(0, 0, 0);
        currentAnimation = null;
        break;
    }
  }

  @Override
  public void periodic() {
    if (currentAnimation != null) {
      candle.animate(currentAnimation, animChannel);
    }
  }

}