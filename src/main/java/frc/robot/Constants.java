package frc.robot;

import java.util.HashMap;
import java.util.Map;

public final class Constants {
    public static class SwerveConstants {
        public static Map<Character, Integer[]> aprilTagRotationValues = new HashMap<Character, Integer[]>() {{
            put('A', new Integer[] {0, 18, 7}); //robot angle, blue alliance tag, red alliance tag
            put('B', new Integer[] {0, 18, 7}); //red alliance right feeder station
            put('C', new Integer[] {60, 17, 8});
            put('D', new Integer[] {60, 17, 8});
            put('E', new Integer[] {120, 22, 9});
            put('F', new Integer[] {120, 22, 9}); //red alliance close left
            put('G', new Integer[] {180, 21, 10}); //red alliance front
            put('H', new Integer[] {180, 21, 10}); //red alliance close right
            put('I', new Integer[] {-120, 20, 11}); //red alliance far right
            put('J', new Integer[] {-120, 20, 11}); //red alliance back
            put('K', new Integer[] {-60, 22, 6}); //red allaince far left
            put('L', new Integer[] {-60, 22, 6}); //blue alliance right feeder station
            
        }};
        
        
    }

    public static class ArmConstants {
        public static double armHomePosition = -0.25;
        public static double armHomePosTolerance = 0.002; 

    
        public static final ArmPosition homePosition = new ArmPosition(0, armHomePosition, 0, 0);
        public static final ArmPosition intakePosition = new ArmPosition(1, -0.308, -5.401, -0.119);
        public static final ArmPosition restPosition = new ArmPosition(2, -0.2, -5.301, 0);        
        public static final ArmPosition L1Position = new ArmPosition(3, -0.082, -0.5342, -0.13); //laster distance: 100, -7.88 yaw for x
        public static final ArmPosition L2Position = new ArmPosition(4,-0.023438, -4.6108, 0.058);
        public static final ArmPosition L3Position = new ArmPosition(5,-0.023438, -19.244, 0.058); 
        public static final ArmPosition L4Position = new ArmPosition(6,0.14, -22.1, 0.265);
        public static final ArmPosition lowAlgaeRemoval = new ArmPosition(7, -0.05, -5.301, 0);
        public static final ArmPosition highAlgaeRemoval = new ArmPosition(8, -0.05, -5.301, 0);
        public static final ArmPosition climbPosition = new ArmPosition(9, -0.3, -2.5, 0.075);
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

    public static class WristConstants {
        public static double wristHomePosition = 0.0;
        public static double wristHomePosTolerance = 0.002;
    }

    public static class ElevatorConstants {
        public static double elevatorHomePosition = 0.0;
        public static double elevatorHomePosTolerance = 0.002;
    }    
}
