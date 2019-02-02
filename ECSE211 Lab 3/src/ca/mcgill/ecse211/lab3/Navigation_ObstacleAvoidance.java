package ca.mcgill.ecse211.lab3;



import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab3.Navigation;
import ca.mcgill.ecse211.lab3.UltrasonicController;
import ca.mcgill.ecse211.lab3.UltrasonicPoller;

public class Navigation_ObstacleAvoidance extends Thread {
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor sensorMotor;
	private Odometer odometer;
    private static final int bandCenter = 25; // Offset from the wall (cm)
	private static final int bandWidth = 2; // Width of dead band (cm)
	
	public Navigation_ObstacleAvoidance(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,EV3LargeRegulatedMotor sensorMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.odometer = odometer;
	}

}
