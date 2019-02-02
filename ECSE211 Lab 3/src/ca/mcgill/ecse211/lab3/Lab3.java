package ca.mcgill.ecse211.lab3;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;


public class Lab3 {

	  // Motor Objects, and Robot related parameters
	  private static final EV3LargeRegulatedMotor leftMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	  private static final EV3LargeRegulatedMotor rightMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	  private static final EV3LargeRegulatedMotor sensorMotor =
		      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));//motor for the us sensor
	//implement the us sensor
	  private static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
	  
	  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	  public static final double WHEEL_RAD = 2.1;
	  public static final double TRACK = 12.12;

	  public static void main(String[] args) throws OdometerExceptions {

	    int buttonChoice;

	    // Odometer related objects
	    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
  
	    Display odometryDisplay = new Display(lcd); // No need to change
	    SampleProvider usDistance = usSensor.getMode("Distance");

	    do {
	      // clear the display
	      lcd.clear();

	      // ask the user whether the motors should drive in a square or float
	      lcd.drawString("< Left | Right >", 0, 0);
	      lcd.drawString("       |        ", 0, 1);
	      lcd.drawString("Simple | Obstacle ", 0, 2);
	      lcd.drawString("Navi  | Avoidance  ", 0, 3);
	      lcd.drawString("       | Navi", 0, 4);

	      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
	    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

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

	      // Start correction if right button was pressed
	      if (buttonChoice == Button.ID_RIGHT) {

            //TODO : OBN
	      
	      }
	      
	      
	    }

	    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
	    System.exit(0);
	  }
	}

