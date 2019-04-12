

/**
 * This class implements color classification to identify the color of the cans. Essentially, it
 * implements the color sensor to obtain the RGB values of the colored cans. Then, the RGB values
 * collected will be normalized by the average over all 3 channels, and compared with the normalized
 * mean value of the color of each can. The color can be identified once the minimum difference in
 * the RGB values is met.
 * 
 * @author1 Ketan Rampurkar
 * @author2 Hao Shu
 * @author3 Carlo D��Angelo
 * @author4 Mohamed Samee
 * @author5 Marie-Lynn Mansour
 * @author6 Cristian Ciungu
 * @version 25-02-2019
 * 
 * 
 */

package ca.mcgill.ecse211.lab5;

import lejos.robotics.SampleProvider;


public class ColorClassification {

  // use an array to collect color data
  private float[] colorData;
  private SampleProvider colorId;

  /**
   * This 2d array stores the mean RGB values for each colored can
   * 
   */
  private final float[][] meanRGB = {// the mean RGB values for cans
      {0.0481924454f, 0.0107023890f, 0.0065211908f}, // red can
      {0.0275519090f, 0.0167073221f, 0.0058014234f}, // yellow can
      {0.0055042342f, 0.0206447724f, 0.0235459889f}, // blue can
      {0.0057257231f, 0.0232034322f, 0.0075382312f} // green can
  };


  /**
   * Constructor to initialize variables
   * 
   * @param float[] colorData
   * @param SampleProvider colorId
   */
  public ColorClassification(float[] colorData, SampleProvider colorId) {
    this.colorData = colorData;
    this.colorId = colorId;



  }

  /**
   * This method receives the position of the detected color and returns the name of the color as a
   * String
   * 
   * @return String
   */
  public String run() {

    int a = findColor(sampleData());// position of the detected color

    if (a != 4) {// when a color is detected

      // use an array to store the color names
      String[] clrName = {"red      ", "yellow   ", "blue     ", "green    "};
      return clrName[a];
    } else {// when no color is detected

      return "no object";
    }


  }


  /**
   * This method normalizes the RGB data collected by the color sensor, and then compare them with
   * the normalized mean RGB values one by one. If the minimum difference is met, the position of
   * the detected color in the 2d array will be returned as an int
   * 
   * @param colorData
   * @return i : int
   */

  public int findColor(float[] colorData) {

    // find the average over all 3 channels
    float average = (float) Math
        .sqrt((Math.pow(colorData[0], 2) + Math.pow(colorData[1], 2) + Math.pow(colorData[2], 2)));

    // normalize R,G,B values
    float nR = colorData[0] / average;
    float nG = colorData[1] / average;
    float nB = colorData[2] / average;
    // use a counter and difference in RGB values to classify the color
    for (int i = 0; i < 4; i++) {
      float deltaR = Math.abs(nR - (meanRGB[i][0] / average));
      float deltaG = Math.abs(nG - (meanRGB[i][1] / average));
      float deltaB = Math.abs(nB - (meanRGB[i][2] / average));


      if (deltaR < 0.5 && deltaG < 0.5 && deltaB < 0.5) {// when the difference is very small

        return i;// return the position of the detected data
      }

    }

    return 4;// otherwise, return a value that means " nothing detected"
  }

  /**
   * This method is used to fetch the RGB values from the color sensor and store them in an array
   * 
   * @return colorData : float[]
   */
  public float[] sampleData() {
    colorId.fetchSample(colorData, 0);
    return colorData;
  }

}