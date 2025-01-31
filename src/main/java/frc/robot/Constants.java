package frc.robot;

import java.util.HashMap;
import java.util.Map;

public final class Constants {
    public static class SwerveConstants {
        public static Map<Character, Double> aprilTagRotationValues = new HashMap<Character, Double>() {{
            put('A', 0.0); //red alliance left feeder station
            put('B', 0.0); //red alliance right feeder station
            put('C', 60.0);
            put('D', 60.0);
            put('E', 120.0);
            put('F', 120.0); //red alliance close left
            put('G', 180.0); //red alliance front
            put('H', 180.0); //red alliance close right
            put('J', -120.0); //red alliance far right
            put('I', -120.0); //red alliance back
            put('K', -60.0); //red allaince far left
            put('L', -60.0); //blue alliance right feeder station
            
        }};
        
    }
}
