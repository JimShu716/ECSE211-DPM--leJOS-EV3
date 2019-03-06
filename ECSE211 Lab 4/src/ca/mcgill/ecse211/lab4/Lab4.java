
package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
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
 * @author Cristian Ciungu
 * @author Hao Shu
 * @version 12-02-2019
 * 
 */
public class Lab4 {

  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  
  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  
  /**
   * The LCD screen.
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();
  
  /**
   * The ultrasonic port.
   */
  public static final Port US_PORT = LocalEV3.get().getPort("S1");
  
  /**
   * The ultrasonic sensor.
   */
  public static final SensorModes US_SENSOR = new EV3UltrasonicSensor(US_PORT);
  
  /**
   * Fetches samples from the ultrasonic sensor.
   */
  public static SampleProvider usDistance = US_SENSOR.getMode("Distance");
  
  /**
   * The color sensor.
   */
  public static final EV3ColorSensor COLOR_SENSOR =
      new EV3ColorSensor(LocalEV3.get().getPort("S2"));

  /**
   * The radius of the wheel.
   */
  public static final double WHEEL_RAD = 2.2;
  
  /**
   * The axle track of the robot.
   */
  public static final double TRACK = 10.6;
  
  /**
   * The speed for the robot to go forward.
   */
  public final static int FORWARD_SPEED = 80;
  
  /**
   * Speed used to move the robot as it localizes.
   */
  public static final int ROTATION_SPEED = 75;
  
  /**
   * The odometer.
   */
  public static Odometer odometer;

  /**
   * Lab 4 main entry point.
   * 
   * @param args
   * @throws OdometerExceptions
   */
  public static void main(String[] args) throws OdometerExceptions {
    int buttonChoice;

    // Odometer related objects
    odometer = Odometer.getOdometer(LEFT_MOTOR, RIGHT_MOTOR, TRACK, WHEEL_RAD);
    
    // initialize the display
    Display odometryDisplay = new Display(LCD);

    do {
      // clear display
      LCD.clear();

      // ask the user whether the motors should use rising or falling edge
      LCD.drawString("< Left | Right >", 0, 0);
      LCD.drawString("       |        ", 0, 1);
      LCD.drawString("Rising |Falling ", 0, 2);
      LCD.drawString(" Edge  |  Edge  ", 0, 3);
      LCD.drawString("       |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
      
    new Thread(odometer).start();
    new Thread(odometryDisplay).start();
    
    if (buttonChoice == Button.ID_LEFT) { // when the user chooses rising edge
      UltrasonicLocalizer.risingEdge(); 
    } else {
      UltrasonicLocalizer.fallingEdge();
    }
    
    buttonChoice = Button.waitForAnyPress(); // start the light localizer after pressing the button
    LightLocalizer.localize();

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);// exit the program after pressing the escape button
  }

}
