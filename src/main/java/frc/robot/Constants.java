package frc.robot;

import java.util.HashMap;
import java.util.Map;

public final class Constants {
    public static class SwerveConstants {
        public static Map<Integer, Double> aprilTagRotationValues = new HashMap<Integer, Double>() {{
            put(1, 45.0); //red alliance left feeder station
            put(2, -45.0); //red alliance right feeder station
            put(3, 120.0);
            put(4, 180.0);
            put(5, 0.0);
            put(6, -60.0); //red alliance close left
            put(7, 0.0); //red alliance front
            put(8, 60.0); //red alliance close right
            put(9, 120.0); //red alliance far right
            put(10, 180.0); //red alliance back
            put(11, -120.0); //red allaince far left
            put(12, -45.0); //blue alliance right feeder station 
            put(13, 45.0); //blue alliance left feeder station
            put(14, 0.0); 
            put(15, 0.0);
            put(16, 0.0);
            put(17, 60.0); //blue alliance close right
            put(18, 0.0); //blue alliance front
            put(19, -60.0); //blue alliance close left
            put(20, -120.0); //blue alliance far left
            put(21, -180.0); //blue alliance back
            put(22, 120.0); //blue alliance far right
            
        }};
        
    }
}
