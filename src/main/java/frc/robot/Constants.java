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
        public static Map<Integer, Integer[]> aprilTagAlgaeData = new HashMap<Integer, Integer[]>() {{
            put(7, new Integer[] {180, 1}); //key is april tag id, values are robot rotation and if there is a high algae or not (1 = high algae, 0 = low algae)
            put(8, new Integer[] {-120, 0}); 
            put(9, new Integer[] {-60, 1});
            put(10, new Integer[] {0, 0});
            put(11, new Integer[] {60, 1});
            put(6, new Integer[] {120, 0}); 
            put(18, new Integer[] {0, 1}); 
            put(17, new Integer[] {60, 0}); 
            put(22, new Integer[] {120, 1});
            put(21, new Integer[] {180, 0});
            put(20, new Integer[] {-120, 1}); 
            put(19, new Integer[] {-60, 0}); 
            
        }};

        
        
    }

    public static class ArmConstants {
        public static double armHomePosition = -0.25;
        public static double armHomePosTolerance = 0.002; 

    
        public static final ArmPosition homePosition = new ArmPosition(0,-0.25, 0, 0);
        public static final ArmPosition intakePosition = new ArmPosition(1, -0.296, -5.401, -0.119);
        public static final ArmPosition restPosition = new ArmPosition(2, -0.2, -5.301, 0);        
        public static final ArmPosition L1Position = new ArmPosition(3, -0.082, -2.0342, -0.13); //laster distance: 100, -7.88 yaw for x
        public static final ArmPosition L2Position = new ArmPosition(4,-0.028438, -4.6108, 0.058);
        public static final ArmPosition L3Position = new ArmPosition(5,-0.028438, -19.244, 0.058); 
        public static final ArmPosition L4Position = new ArmPosition(6,0.149, -21.96, 0.28);
        public static final ArmPosition lowAlgaeRemoval = new ArmPosition(7, -0.05, -5.301, 0);
        public static final ArmPosition highAlgaeRemoval = new ArmPosition(8, 0.15, -5.301, 0.267);
        public static final ArmPosition climbPosition = new ArmPosition(9, -0.309, -2.3, 0.065);
    }
    public static class ArmPosition{
        public double armTargetAngle = -0.25; //home angle
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
