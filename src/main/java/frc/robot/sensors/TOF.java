package frc.robot.sensors;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;

public class TOF {
  private CAN can;
  private CANData rxData;
  private static final int DEVICE_NUMBER = 50;
  private static final int API_ID_TX = 0x002; // API ID for roboRIO -> ESP8266
  private static final int API_ID_RX = 0x001; // API ID for ESP8266 -> roboRIO
  private int distance = 0;

  public TOF() {
    can = new CAN(DEVICE_NUMBER);  // deviceType and manufacturer are defaulted to TEAM USABLE values
    rxData = new CANData();
    System.out.println("CAN Distance Sensor initialized.");
  }

  /**
   * This method needs to be called every robotPeriodic to ensure CAN messages are recevived as distance updates occur
   */
  public void periodicUpdate() {
    if (can.readPacketNew(API_ID_RX, rxData)) {
      distance = (rxData.data[0] << 8) | (rxData.data[1] & 0xFF); // Combine high/low bytes
    }
  }

  /**
   * Returns the current distance measurement from the sensor in mm
   */
  public int getDistance() {
    return distance;
  }
}