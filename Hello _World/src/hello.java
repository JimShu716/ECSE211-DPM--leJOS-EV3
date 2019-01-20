import lejos.hardware.Button;
public class hello {

	public static void main(String[] args) {
		//print hello world on the LED screen
		System.out.println("HelloWorld");
		//wait for a button press before exiting.
		Button.waitForAnyPress();
	
	}
}
