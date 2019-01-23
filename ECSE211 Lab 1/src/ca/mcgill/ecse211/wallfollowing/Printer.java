/**
 * This class implements a thread to print information on 
 * the EV3 screen. It first asks the user to either choose
 * Bang-Bang or P-type to control the tribot's distance from
 * the wall. Since it runs concurrently with the UltrasonicPoller
 * thread, it is set to sleep for 200 ms so that the distance
 * sampling is done as smoothly as possible . Therefore, distance
 * readings are printed on the screen at a frequency of 5 Hz. 
 * 
 * @author1 Cristian Ciungu
 * @author2 Hao Shu
 * @version 2019-01-22
 * 
 * 
 */



// declare package

package ca.mcgill.ecse211.wallfollowing;


// imports 

import lejos.hardware.Button;

import lejos.hardware.ev3.LocalEV3;

import lejos.hardware.lcd.TextLCD;



public class Printer extends Thread {

  // parameters 
	
  private UltrasonicController cont;

  private final int option;


  // define constructor the printer 
  
  public Printer(int option, UltrasonicController cont) {

    this.cont = cont;

    this.option = option;

  }

  // obtain access to EV3 screen 

  public static TextLCD t = LocalEV3.get().getTextLCD(); 


  // Printer thread starts 
  
  public void run() {

	// thread is operating continuously
	  
    while (true) { 

      t.clear(); // clear screen just in case something is already displayed

      t.drawString("Controller Type is... ", 0, 0); // print header

      if (this.option == Button.ID_LEFT) // user chooses Bang-Bang 

        t.drawString("Bang-Bang", 0, 1); 

      else if (this.option == Button.ID_RIGHT) // user chooses P-type

        t.drawString("P-type", 0, 1);

      t.drawString("US Distance: " + cont.readUSDistance(), 0, 2); // print last US distance reading


      // catch thread exceptions 
      
      try {

        Thread.sleep(200); // sleep for 200 ms 

      } catch (Exception e) {

        System.out.println("Error: " + e.getMessage());

      }

    }

  } // end run 


  // method to print the main menu on the EV3 screen 
  
  public static void printMainMenu() { 

    t.clear(); // just in case there is something on the screen already 

    t.drawString("Left button for Bang-Bang", 0, 0);

    t.drawString("Right button for P-type", 0, 1);

  } // end printMainMenu

} // end Printer 