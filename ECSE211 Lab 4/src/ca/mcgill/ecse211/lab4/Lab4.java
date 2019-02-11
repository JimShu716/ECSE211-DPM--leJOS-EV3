
/**
	 * This class contains the main method which implements the 
	 * odometer and the navigation systems on the EV3 robot. It 
	 * first initializes the robot's motors and ultrasonic sensor, 
	 * defines hardware constants such as the axle track, and 
	 * prompts the user to execute a simple navigation or an obstacle 
	 * avoidance navigation. Then, threads are instantiated for the 
	 * odometer and the navigator. 
	 * 
	 * @author1 Cristian Ciungu
	 * @author2 Hao Shu
	 * 
	 * @version 05-02-2019
	 * 
	 */
package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
public class Lab4 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 10.50;
  public static boolean isRisingEdge; // makes it easier to know if falling edge or rising edge method (from lightLocalizer) should be used

	public static void main(String[] args) throws OdometerExceptions {
		int buttonChoice;

		Odometer odometer = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		UltrasonicLocalizer ultrasonicLoc = new UltrasonicLocalizer(leftMotor, rightMotor,odometer);
        LightLocalizer lightLoc = new LightLocalizer(leftMotor, rightMotor, odometer);


		Display odometryDisplay = new Display(lcd);
		do {
			// clear the display
			lcd.clear();

			// ask the user whether rising edge or falling edge is used
			lcd.drawString("< Left |  Right >", 0, 0);
			lcd.drawString("       |         ", 0, 1);
			lcd.drawString("Rising |  Falling", 0, 2);
			lcd.drawString("edge   |  edge   ", 0, 3);
			lcd.drawString("       | 		 ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
		  
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			Thread lightThread = new Thread(lightLoc);
			ultrasonicLoc.risingEdge();// use rising edge
			buttonChoice = Button.waitForAnyPress();
		    lightThread.start();

		} else {
			
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			Thread lightThread = new Thread(lightLoc);
			
			

			
			buttonChoice = Button.waitForAnyPress();
			lightThread.start();

		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}