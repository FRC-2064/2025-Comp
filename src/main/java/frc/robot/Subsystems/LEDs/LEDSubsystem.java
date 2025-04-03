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
        // Clear any animations and turn off LEDs
        candle.clearAnimation(animChannel);
        candle.setLEDs(0, 0, 0); // Set LEDs to off (black)
        currentAnimation = null;
        break;
        
      case LIGHTBAR:
        // Ensure lightbar is fully lit up (white color)
        candle.clearAnimation(animChannel);

        candle.setLEDs(255, 255, 255); // Set LEDs to white (full brightness)
        currentAnimation = null; // No animation needed here
        break;

      case HAS_PIECE:
        animChannel = 1;
        // Fire animation for "has piece"
        currentAnimation = new FireAnimation(0.5, 0.7, LEDConstants.NUM_LEDS, 0.8, 0.5, false, 0);
        break;

      case NO_PIECE:
        animChannel = 2;
        // Twinkle animation for "no piece"
        currentAnimation = new TwinkleAnimation(30, 70, LEDConstants.NUM_LEDS, 0, 0.4, LEDConstants.NUM_LEDS, TwinkleAnimation.TwinklePercent.Percent42, 0);
        break;

      case CLIMBED:
        animChannel = 3;
        // Rainbow animation for "climbed"
        currentAnimation = new RainbowAnimation(1, 0.7, LEDConstants.NUM_LEDS, false, 0);
        break;

      default:
        break;
    }
    
    // Only animate if there's an animation set
    if (currentAnimation != null) {
      candle.animate(currentAnimation);
    }
  }

}