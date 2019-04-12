package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.searchzonelocator.SearchZoneLocator;

import java.text.DecimalFormat;

import ca.mcgill.ecse211.canlocator.CanDetect;
import ca.mcgill.ecse211.canlocator.CanLocator;
import ca.mcgill.ecse211.localization.*;

public class Lab5 {

	// Motor and Sensor Ports
	private static final EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor SENSOR_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD LCD = LocalEV3.get().getTextLCD();
	private static final Port US_PORT = LocalEV3.get().getPort("S1");
	private static final Port CS_PORT = LocalEV3.get().getPort("S4");
	private static final Port CS_FRONT_PORT = LocalEV3.get().getPort("S2");
	
	public static final int LLx = 2, LLy = 2, URx = 5, URy = 5; // SearchZone description
	private static final int SC = 0; //Starting corner
	private static final int TR = 1; //target color
	
	//Robot related parameters
	public static final double WHEEL_RAD = 2.2;
	//public static final double TRACK = 13.5;
	public static final double TRACK = 13.4;
	
	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(LEFT_MOTOR, RIGHT_MOTOR, TRACK, WHEEL_RAD);
		Navigation navigator = new Navigation(LEFT_MOTOR,RIGHT_MOTOR);
		Display odometryDisplay = new Display(LCD); 

		// Ultrasonic sensor
		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(US_PORT);
		SampleProvider usDistance = usSensor.getMode("Distance");
		float[] usData = new float[usDistance.sampleSize()];
		
		// Color Sensor (Localization)
		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes csSensor = new EV3ColorSensor(CS_PORT);
		SampleProvider csLineDetector = csSensor.getMode("Red");
		float[] csData = new float[csLineDetector.sampleSize()];
		
		 // Color Sensor (Color Classification)
        @SuppressWarnings("resource") // Because we don't bother to close this resource
        SensorModes clrSensor = new EV3ColorSensor(CS_FRONT_PORT);
        SampleProvider colorId =  clrSensor.getMode("RGB");
        float[] colorData = new float[colorId.sampleSize()];
        colorId.fetchSample(colorData, 0);
		
        ColorClassification ClrClassify= new ColorClassification (colorData, colorId);
        CanDetect canDetect = new CanDetect(SENSOR_MOTOR, ClrClassify);
        
		do {
			
			LCD.clear();

			// ask the user whether robot should do Rising Edge or Falling Edge
			LCD.drawString("< Left | Right >", 0, 0);
			LCD.drawString("       |        ", 0, 1);
			LCD.drawString(" Field |Color   ", 0, 2);
			LCD.drawString(" Test  |Class.  ", 0, 3);
			LCD.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

			if (buttonChoice == Button.ID_LEFT) { // do Field Test
				
				Thread odoThread = new Thread(odometer);
				odoThread.start();

				Thread odoDisplayThread = new Thread(odometryDisplay);
				odoDisplayThread.start();
				
				// Localization (Ultrasonic and Light)
				UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer(LEFT_MOTOR, RIGHT_MOTOR, usDistance, usData);
				LightLocalizer lightLocalizer = new LightLocalizer(LEFT_MOTOR, RIGHT_MOTOR, csLineDetector, csData, navigator);

				ultrasonicLocalizer.fallingEdge();

				lightLocalizer.moveClose();
				lightLocalizer.lightLocalize(0,0);
				
				// Search Zone Locator
				SearchZoneLocator searchZonelocator = new SearchZoneLocator(SC, LLx, LLy, lightLocalizer, navigator);
				searchZonelocator.goToSearchZone();
				
				Sound.beep(); // Must BEEP after navigation to search zone is finished
				
				CanLocator canLocator = new CanLocator(canDetect, usDistance, usData, 
			navigator,lightLocalizer, TR, LLx, LLy, URx, URy);
				canLocator.RunLocator();
				
				
			} else {
				LCD.clear();
				
//				Thread odoThread = new Thread(odometer);
//				odoThread.start();
//
//				Thread odoDisplayThread = new Thread(odometryDisplay);
//				odoDisplayThread.start();
//				LightLocalizer lightLocalizer = new LightLocalizer(LEFT_MOTOR, RIGHT_MOTOR, csLineDetector, csData, navigator);
//				CanLocator canLocator = new CanLocator(canDetect, usDistance, usData, 
//						navigator,lightLocalizer, TR, LLx, LLy, URx, URy);
//				canLocator.RunLocator();
				canDetect.run();
				/*while(true) {
					
					if (ClrClassify.run() !="no object") {	//if there is a can detected
				    LCD.drawString("Object Detected", 1,1);
				    LCD.drawString(ClrClassify.run(), 1, 2);
				    
				    } else {
				    	LCD.clear();
				    }

//				    colorId.fetchSample(colorData, 0);
//				    LCD.drawString("R: " + colorData[0], 1, 3);
//			        LCD.drawString("G: " + colorData[1], 1, 4);
//			        LCD.drawString("B: " + colorData[2], 1, 5);
				  
				 }*/
				
			}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}

}
