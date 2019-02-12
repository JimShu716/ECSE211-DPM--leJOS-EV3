
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
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;



public class Lab4 {

    // Motor Objects, and Robot related parameters
    private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
    private static final TextLCD lcd = LocalEV3.get().getTextLCD();
    private static final Port usPort = LocalEV3.get().getPort("S1");
    private static boolean Risingorfalling = true;

    //Robot related parameters
    public static final double WHEEL_RAD = 2.2;
    public static final double TRACK = 10.50;

    public static void main(String[] args) throws OdometerExceptions {

        int buttonChoice;

        // Odometer related objects
        Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
        Display odometryDisplay = new Display(lcd); 

        @SuppressWarnings("resource") // Because we don't bother to close this resource
        // Instance  ultrasonicsensor 
        SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
        // usDistance fetch samples from this instance
        SampleProvider usDistance = ultrasonicSensor.getMode("Distance");
        
        UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(odometer, leftMotor, rightMotor, Risingorfalling, usDistance);
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

        if (buttonChoice == Button.ID_LEFT) {
          
            Thread odoThread = new Thread(odometer);
            odoThread.start();
            Thread odoDisplayThread = new Thread(odometryDisplay);
            odoDisplayThread.start();

            Thread lightThread = new Thread(lsLocalizer);
            usLocalizer.risingEdge();// use rising edge
            buttonChoice = Button.waitForAnyPress();//start the light localizer after pressing the button
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

        while (Button.waitForAnyPress() != Button.ID_ESCAPE)
            ;
        System.exit(0);
    }

}