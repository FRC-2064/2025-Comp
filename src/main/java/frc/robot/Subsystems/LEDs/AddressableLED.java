package frc.robot.Subsystems.LEDs;
// package frc.robot;

// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;

// public class AddressableLED {
//     private AddressableLED led;
//     private final AddressableLEDBuffer ledBuffer;
//     private Timer timer;

//     public AddressableLED(int port, in numberOfLEDs){
//         led = new AddressableLED(port);
//         led.setLength(numberOfLEDs);
//         ledBuffer = new AddressableLEDBuffer(numberOfLEDs);
//         led.setData(ledBuffer);

//         timer = new Timer();
//         timer.start();
//     }

//     public AddressableLED getLED() {
//         return led;
//     }

//     public void start() {
//         led.start();
//     }

//     public void stop() {
//         led.stop();
//     }

//     public void setColor(Color8Bit color) {
//         for (var i = 0; i < ledBuffer.getLength(); i++) {
//             ledBuffer.setRGB(i, color.red, color.green, color.blue);
//         }
//     }

//     public void setColor(Color color) {
//         for (var i = 0; i < ledBuffer.getLength(); i++) {
//             ledBuffer.setLED(i, color);
//         }
//     }

//     public void setColor(Color8Bit color, int index) {
//         ledBuffer.setRGB(index, color.red, color.green, color.blue);
//     }

//     public void commitColor() {
//         led.setData(ledBuffer);
//     }

//     public void left(int startPoint, int endPoint) {
//         if ((int) state.get("loop") == 0) {
//             Color8Bit temp = ledBuffer.getLED8Bit(startPoint);
//             for (var i = startPoint + 1; i < endPoint; i++) {
//                 Color8Bit color = ledBuffer.getLED8Bit(i);
//                 ledBuffer.setRGB(i - 1, color.red, color.green, color.blue);
//             }
//             ledBuffer.setRGB(endPoint - 1, temp.red, temp.green, temp.blue);
//         }
//     }

//     public void right(int startPoint, int endPoint) {
//         if ((int) state.get("loop") == 0) {
//             Color8Bit temp = ledBuffer.getLED8Bit(endPoint - 1);
//             for (var i = endPoint - 2; i >= startPoint; i--) {
//                 Color8Bit color = ledBuffer.getLED8Bit(i);
//                 ledBuffer.setRGB(i + 1, color.red, color.green, color.blue);
//             }
//             ledBuffer.setRGB(startPoint, temp.red, temp.green, temp.blue);
//         }
//     }

//     public void setInitTurning(Color8Bit color, int startPoint, int endPoint) {
//         // create the dashes in red
//         state.put("loop", 0);
//         int on = 1;
//         for (var i = startPoint; i < endPoint; i++) {
//             if (on >= 6) {
//                 ledBuffer.setRGB(i, 0, 0, 0);
//             } else {
//                 ledBuffer.setRGB(i, color.red, color.green, color.blue);
//             }
//             if (on == 10) {
//                 on = 0;
//             }
//             on += 1;
//         }
//         commitColor();

//     }

//     public void breathe(Color8Bit color, boolean enabled) {
//         if (state.get("state") != "breathe") {
//             state.clear();
//             state.put("state", "breathe");
//         }

//         double amplitude = (enabled ? 0.74 : 0.04) * (1 + Math.sin(timer.get())) / 2 + .01;

//         double red = (double) amplitude * color.red / 100;
//         double green = (double) amplitude * color.green / 100;
//         double blue = (double) amplitude * color.blue / 100;
//         setColor(new Color(red, green, blue));

//         commitColor();
//     }

//     public void rainbow() {
//         if (state.get("state") != "rainbow") {
//             state.clear();
//             state.put("state", "rainbow");
//             state.put("rainbowFirstPixelHue", 0);
//         }

//         int rainbowFirstPixelHue = (int) state.get("rainbowFirstPixelHue");
//         // For every pixel
//         for (var i = 0; i < ledBuffer.getLength(); i++) {
//             // Calculate the hue - hue is easier for rainbows because the color
//             // shape is a circle so only one value needs to precess
//             final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
//             // Set the value
//             ledBuffer.setHSV(i, hue, 255, 128);
//         }
//         // Increase by to make the rainbow "move"
//         rainbowFirstPixelHue += 3;
//         // Check bounds
//         rainbowFirstPixelHue %= 180;

//         state.put("rainbowFirstPixelHue", rainbowFirstPixelHue);
//         commitColor();
//     }

//     public void cautionBlink(Color8Bit color) {
//         if (state.get("state") != "cautionBlink") {
//             state.clear();
//             state.put("state", "cautionBlink");
//             state.put("even", 0);
//         }
//         for (var i = 0; i < ledBuffer.getLength(); i++) {
//             if ((int) state.get("even") < 4) {
//                 if (i % 4 >= 2) {
//                     ledBuffer.setRGB(i, color.red, color.green, color.blue);
//                 } else {
//                     ledBuffer.setRGB(i, 0, 0, 0);
//                 }
//             } else {
//                 if (i % 4 >= 0) {
//                     ledBuffer.setRGB(i, 0, 0, 0);
//                 } else {
//                     ledBuffer.setRGB(i, color.red, color.green, color.blue);
//                 }
//             }
//         }

//         state.put("even", (int) state.get("even") + 1);

//         if ((int) state.get("even") == 8) {
//             state.put("even", 0);
//         }

//         commitColor();
//     }
// }
