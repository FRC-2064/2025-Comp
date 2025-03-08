package frc.robot.Subsystems;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class DistanceSensorUSB {
  private Pattern distancePattern;
  private SerialPort serialPort;
  private boolean bPortFound = false;
  private int distance = 0;

  public DistanceSensorUSB() {
    Port[] available_ports = { Port.kUSB, Port.kUSB1, Port.kUSB2 };
    // Scan the USB ports for the device
    for (Port port : available_ports) {
      try {
        serialPort = new SerialPort(115200, port);
        //serialPort.enableTermination();
        bPortFound = true;
        System.out.println("DistanceSensorUSB connected on " + port);
        break;
      } catch (Exception ex) {
      }
    }
    if (!bPortFound) {
      System.err.println("UNABLE TO CONNECT TO DistanceSensorUSB");
    }
    distancePattern = Pattern.compile("\\{(\\d+)\\}");
  }

  /**
   * This method needs to be called every robotPeriodic to ensure CAN messages are
   * recevived as distance updates occur
   */
  public void periodicUpdate() {
    if (bPortFound) {
      String data = serialPort.readString();
      if (data.length() > 0) {
        Matcher m = distancePattern.matcher(data);
        if(m.find()) {
          distance = Integer.parseInt(m.group(1));
        }
      }
    }
  }

  /**
   * Returns the current distance measurement from the sensor in mm
   */
  public int getDistance() {
    return distance;
  }
}
