
package ca.mcgill.ecse211.lab4;



import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;

public class LightLocalizer extends Thread {
  
  private EV3LargeRegulatedMotor leftMotor, rightMotor;

  private Odometer odometer;
  
  private SampleProvider color;
  private float[] lightData;
  private float light_received;
  
  
  
  public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {

  this.leftMotor = leftMotor;
  this.rightMotor = rightMotor;
  this.odometer = odometer;

  EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
  color = lightSensor.getMode("Red");
  lightData = new float[lightSensor.sampleSize()];
}
  



  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param Lab3.WHEEL_RAD
   * @param distance
   * @return (int)distance converted
   * 
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * This method allows the conversion of an angle to the total distance that the robot needs to
   * cover.
   * 
   * @param radius
   * @param wideltaTh
   * @param angle
   * @return (int)angle converted
   * 
   */

  private static int convertAngle(double radius, double wideltaTh, double angle) {
    return convertDistance(radius, Math.PI * wideltaTh * angle / 360.0);
  }

}
