package frc.robot.sensors;
import edu.wpi.first.wpilibj.I2C;

public class VL53L0X {
    private static final int VL53L0X_ADDRESS = 0x29;
    private I2C i2c;

    public VL53L0X(I2C.Port port){
        i2c = new I2C(port, VL53L0X_ADDRESS);
        initSensor();
    }

    private void initSensor(){
        //init sensor
    }

    public int getDistance(){
        byte[] buffer = new byte[2];
        boolean success = i2c.read(0x14, 2, buffer);
        if(success){
            int distance = ((buffer[0] & 0xFF) << 8) | (buffer[1] & 0xFF);
            System.out.println("Distance:" + (distance / 25.4) + "inches");
            return distance; 
        }
        else{
            System.out.println("Failed to read distance");
            return -1; 
        }
    }
}
