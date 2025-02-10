package frc.robot.Utils;

public class TOF {
    double port;

    public TOF(double port) {
        this.port = port;
    }

    public double getDistance() {
        return port;
    }
}
