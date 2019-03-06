
/**
 * This class contains the main method which implements the hardware required to execute the fourth
 * laboratory. The purpose of laboratory 4 is to localize the robot given an arbitrary starting
 * position. In other words, it is placed at a starting position, and it has to gather data using
 * the ultrasonic sensor and the light sensor to orient itself towards the origin of the grid
 * system, i.e (0,0). A reference system is defined in terms of x,y and theta in order to localize
 * the robot. The UltrasonicLocalizer class is first implemented to orient the robot at the required
 * heading. Then, upon the user's prompt, the LightLocalizer class is implemented to drive the robot
 * to the origin of the grid system.
 * 
 * @author1 Cristian Ciungu
 * @author2 Hao Shu
 * @version 12-02-2019
 * 
 */
package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;



public class Lab4 {

  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  private static final Port usPort = LocalEV3.get().getPort("S1");

  // Robot related parameters
  public static final double WHEEL_RAD = 2.2;// the radius of the wheel
  public static final double TRACK = 10.6;// the axel track of the robot

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
    // initialize the display
    Display odometryDisplay = new Display(lcd);

    @SuppressWarnings("resource") // Because we don't bother to close this resource
    // Instance ultrasonicsensor
    SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
    // usDistance fetch samples from this instance
    SampleProvider usDistance = ultrasonicSensor.getMode("Distance");

    UltrasonicLocalizer usLocalizer =
        new UltrasonicLocalizer(odometer, leftMotor, rightMotor, usDistance);
    LightLocalizer lsLocalizer = new LightLocalizer(odometer, leftMotor, rightMotor);


    do {
      // clear display
      lcd.clear();

      // ask the user whether the motors should use rising or falling edge
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("       |        ", 0, 1);
      lcd.drawString("Rising |Falling ", 0, 2);
      lcd.drawString(" Edge  |  Edge  ", 0, 3);
      lcd.drawString("       |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) {// when the user choose rising edge

      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();

      Thread lightThread = new Thread(lsLocalizer);
      usLocalizer.risingEdge();// use rising edge
      buttonChoice = Button.waitForAnyPress();// start the light localizer after pressing the button
      lightThread.start();

    } else {

      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();

      usLocalizer.fallingEdge();// use falling edge
      buttonChoice = Button.waitForAnyPress();
      lsLocalizer.run();

    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);// exit the program after pressing the escape button
  }

}
