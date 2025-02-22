// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
/** 
 * A singleton class to control LED strips. Singleton means we allow only one instance of the class.
 */
public class LEDStrip {
    
    final int NUM_LEDS = 60;    
    private int rainbowFirstHue = 0;
    public static enum LEDModes {
        OFF,
        SOLIDGREEN,
        BLINKGREEN,
        SOLIDRED,
        SOLIDBLUE,
        RAINBOW,
        ROBOTDISABLEDPATTERN, 
        MANUAL
      }
    
    private LEDModes LEDMode;
    private int blinkCnt = 0;

    AddressableLED addressableLED = new AddressableLED(1); //should be PWM location
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(NUM_LEDS);
    private double intervalSeconds= 0.07; 
    private double blinkTime = 3;
	private boolean blinkLEDon = true;
	private double lastChange = 0;
    private int diagnosticPattern;
    
    // diagnostic segments 
    /** Left segment 1 (starts at the top)*/  
    public static final int elbowDiag = 0X01;
    /** Left segment 2 */
    public static final int shoulderDiag = 0X02;
    /** Left segment 3*/
    public static final int gyroDiag = 0X04;
    /** Left segment 4 */
    public static final int limelightDiag = 0X08;
    /** Left segment 5 (bottom) */
    public static final int apriltagDiag = 0X10;
    /** Right segment 1 (starts at bottom) */
    public static final int swerve1Diag = 0X20; // needs position on robot?
    /** Right segment 2 */
    public static final int swerve2Diag = 0X40; // ^
    /** Right segment 3*/
    public static final int swerve3Diag = 0X80; // ^
    /** Right segment 4 */
    public static final int swerve4Diag = 0X100; // ^
    /** All segments blue */
    public static final int allClear = 0X1FF;

    private static final LEDStrip instance = new LEDStrip();

      // private constructor so clients can't use it
    private LEDStrip(){
        addressableLED.setLength(ledBuffer.getLength());

        // Set the data
        addressableLED.setData(ledBuffer);
        addressableLED.start();
        LEDMode = LEDModes.SOLIDBLUE; // ??? set these OFF to start
    }

    public static LEDStrip getInstance(){
                return instance;
    }
    
    private void setColour(int r,int g, int b){
        for (int index = 0; index < ledBuffer.getLength(); index++){
			ledBuffer.setRGB(index, r, g, b);
		}
    }

    private void blinkLEDs(int r, int g,int b){
        double timestamp = Timer.getFPGATimestamp();
		if (timestamp- lastChange > intervalSeconds){
		    blinkLEDon = !blinkLEDon;
			lastChange = timestamp;
            blinkCnt++;
		}
		if (blinkLEDon){
			setColour(r, g, b);
		} else {
			setColour(0, 0, 0);
		}
        if (blinkCnt > 20) {
            setLEDMode(LEDModes.SOLIDGREEN);
        }
    }

    private void rainbow(){
        int currentHue;
		for (int index = 0; index < ledBuffer.getLength(); index++){
			currentHue = (rainbowFirstHue + (index * 180 / ledBuffer.getLength())) % 180;
			ledBuffer.setHSV(index, currentHue, 255, 128);
		}

		rainbowFirstHue = (rainbowFirstHue + 3) % 180;
    }
    public void setStripSection(int section, int r, int g, int b) {
        switch(section) {
            case 0:
                for (int i = 0; i <= 9; i++) {
                    ledBuffer.setRGB(i, r, g, b);
                }
                for (int i = 50; i <= 59; i++) {
                    ledBuffer.setRGB(i, r, g, b);
                }
            break;

            case 1:
                for (int i = 10; i <= 19; i++) {
                    ledBuffer.setRGB(i, r, g, b);
                }
                for (int i = 40; i <= 49; i++) {
                    ledBuffer.setRGB(i, r, g, b);
                }
            break;

            case 2:
                for (int i = 20; i <= 28; i++) {
                    ledBuffer.setRGB(i, r, g, b);
                }
                for (int i = 30; i <= 39; i++) {
                    ledBuffer.setRGB(i, r, g, b);
                }
            break;
        }
    }
    public void setTopLEDS(int r, int g, int b) {
        ledBuffer.setRGB(29, r, g, b);
        ledBuffer.setRGB(59, r, g, b);
    }

    /*
     *  For the diagnostic checks, we split diagnostics across 2 strips of 30 leds.
     *  Strips were divided into segments of LEDs, 1 segment per diagnostic, none spanning over multiple strips.
     *  
     */
    public void diagnosticLEDmode(){
        if (diagnosticPattern == allClear) { // if all is good, go entirely blue
            for(int i = 0;i<ledBuffer.getLength();i++)
                ledBuffer.setRGB(i, 0, 0, 255); 
        }
	    else { // Something other than "all is well".  Light up the required segments that have a 0 bit in bval

            // start with the whole strip red
            for(int i = 0;i<ledBuffer.getLength();i++)
                ledBuffer.setRGB(i, 255, 0, 0); 

            // First strip of LEDS, 6 leds per diagnostic state, 
            // represents shoulder/elbow/gryo/limelight/april tag camera (not necessarily in that order) 
            if((diagnosticPattern & elbowDiag) != 0) {
                for(int i = 0;i<=5;i++)
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            if((diagnosticPattern & shoulderDiag) != 0) {
                for(int i = 6;i<=11;i++) 
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            if((diagnosticPattern & gyroDiag) != 0) {
                for(int i = 12;i<=17;i++) 
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            if((diagnosticPattern & limelightDiag) != 0) {
                for(int i = 18;i<=23;i++) 
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            if((diagnosticPattern & apriltagDiag) != 0) {
                for(int i = 24;i<=29;i++) 
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            // Second LED strip, 7 or 8 leds per diagnostic check
            // represents each each swerve module
            if((diagnosticPattern & swerve1Diag) != 0) {
                for(int i = 30;i<=36;i++) 
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            if((diagnosticPattern & swerve2Diag) != 0) {
                for(int i = 37;i<=44;i++) 
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            if((diagnosticPattern & swerve3Diag) != 0) {
                for(int i = 45;i<=51;i++) 
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            if((diagnosticPattern & swerve4Diag) != 0){
                for(int i = 52; i<=59;i++)
                    ledBuffer.setRGB(i, 0, 255, 0);
            }
	    }
        //setLEDMode(LEDModes.ROBOTDISABLEDPATTERN);
    }

    public void setDiagnosticPattern(int binaryVal){
        diagnosticPattern = binaryVal;
    }

    public void setLEDMode(LEDModes inputLEDMode){
        LEDMode = inputLEDMode;
        blinkCnt = 0;
    }

    public void periodic(){
        setLEDMode(LEDModes.SOLIDBLUE);
        switch(LEDMode){
            case OFF:
                setColour(0,0,0);
                break;
            case SOLIDGREEN:
                setColour(0, 255, 0);
                break;
            case SOLIDRED:
                setColour(255, 0, 0);
                break;
            case SOLIDBLUE:
                setColour(0, 0, 255);
                break;
            case BLINKGREEN:
                blinkLEDs(0, 255, 0);
                break;
            case RAINBOW:
                rainbow();
                break;
            case ROBOTDISABLEDPATTERN:
                diagnosticLEDmode();
                break;
            case MANUAL:

            break;
        }
        addressableLED.setData(ledBuffer);
    }
}