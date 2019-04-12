
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class touch_sensor {
//
//  private static final Port usPort = LocalEV3.get().getPort("S1");
//  

  
  static Port portTouch = LocalEV3.get().getPort("S2");// 
  static SensorModes myTouch = new EV3TouchSensor(portTouch);// 2. Get sensor instance 
  static SampleProvider myTouchStatus = myTouch.getMode(0);// 3. Get sample provider 
  static float[] sampleTouch = new float[myTouchStatus.sampleSize()]; 
  

  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final EV3LargeRegulatedMotor clampMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  
  
  private static final int FORWARD_SPEED = 550;
  private static final int ROTATE_SPEED = 175;
  private static final int ROTATE_AMOUNT = 200;
  
  
  public static void main(String[] args) throws InterruptedException  {
    int buttonChoice;
    buttonChoice = Button.waitForAnyPress(); 
    
    if (buttonChoice == Button.ID_LEFT || buttonChoice == Button.ID_RIGHT) {
      
   
        clampMotor.setAcceleration(500);
        clampMotor.setSpeed(ROTATE_SPEED);
        clampMotor.rotate(ROTATE_AMOUNT);
        

        
        Thread.sleep(1000);
        
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED); 
      leftMotor.forward();
      rightMotor.forward();
    while (true) { 
   
      
       myTouchStatus.fetchSample(sampleTouch, 0);
       float weight= sampleTouch[0];
      System.out.println( weight);
      if(weight == 1.00){
        Sound.beep();

        leftMotor.stop(true);
        rightMotor.stop(true);
      }
    }


}
    
}}
