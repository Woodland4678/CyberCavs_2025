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
    public static class ArmConstants {
        public static final ArmPosition homePosition = new ArmPosition(0,-90, 0, 0);
        public static final ArmPosition intakePosition = new ArmPosition(1, -0.295, -6.201, -0.119);
        public static final ArmPosition restPosition = new ArmPosition(2, -0.2, -5.301, 0);        
        public static final ArmPosition L1Position = new ArmPosition(3, -0.119, -0.67, -0.2276);
        public static final ArmPosition L2Position = new ArmPosition(4,-0.023438, -4.6108, 0.058);
        public static final ArmPosition L3Position = new ArmPosition(5,-0.023438, -19.244, 0.058);
        public static final ArmPosition L4Position = new ArmPosition(6,0.111, -22.1, 0.223);
    }
    public static class ArmPosition{
        public double armTargetAngle = -90; //home angle
        public double elevatorTarget = 0; //home position 
        public double wristTarget = 0; //TODO determine home position
        public int positionID = 0;
        public ArmPosition(int positionID, double armTargetAngle, double elevatorTarget, double wristTarget) {
          this.armTargetAngle = armTargetAngle;
          this.elevatorTarget = elevatorTarget;
          this.wristTarget = wristTarget;
          this.positionID = positionID;
        }
      }
}
