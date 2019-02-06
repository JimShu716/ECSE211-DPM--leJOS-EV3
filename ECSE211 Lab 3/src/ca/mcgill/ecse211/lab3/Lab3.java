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





package ca.mcgill.ecse211.lab3;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	
	
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 10.38;
	public static final int bandCenter = 25;

	
	
	// main program 
	public static void main(String[] args) throws OdometerExceptions {
		
	    
		// Odometer related objects
		final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Navigation_ObstacleAvoidance navigationObstacle = new Navigation_ObstacleAvoidance(leftMotor, rightMotor);

		
		Display odometryDisplay = new Display(lcd);

		// clear the display
		lcd.clear();

		 int buttonChoice;
		do {
		      // clear the display
		      lcd.clear();
		
		      // ask the user whether odometery correction should be run or not
		      lcd.drawString("< Left | Right >", 0, 0);
		      lcd.drawString("       |        ", 0, 1);
		      lcd.drawString("Simple | Obstacle ", 0, 2);
		      lcd.drawString("Navi   | Avoidance  ", 0, 3);
		      lcd.drawString("       | Navi", 0, 4);

		
		      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		  	}	while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		  

	    if (buttonChoice == Button.ID_LEFT) {
	    	Navigation navigation = new Navigation( leftMotor, rightMotor,odometer);

			// Start odometer and display threads
			Thread odoThread = new Thread(odometer);
			odoThread.start();

			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			
			// start navigation 
			
			Thread NavigationThread = new Thread(navigation);
			NavigationThread.start();    
	  
	    } else {
	      // clear the display
	      lcd.clear();

	      // Start odometer and display threads
	      Thread odoThread = new Thread(odometer);
	      odoThread.start();
	      Thread odoDisplayThread = new Thread(odometryDisplay);
	      odoDisplayThread.start();

	      // Start obstacle avoidance if right button was pressed
	      if (buttonChoice == Button.ID_RIGHT) {
				  
				  Thread navigationNoThread = new Thread(navigationObstacle);
				  navigationNoThread.start();
				} 
	    }
	
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);

	} // end main
}