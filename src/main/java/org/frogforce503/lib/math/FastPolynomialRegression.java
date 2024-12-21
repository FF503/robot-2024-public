package org.frogforce503.lib.math;

import java.util.HashMap;
import java.util.TreeSet;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * The {@code FastPolynomialRegression} class performs a linear regression
 * between every two points on an set of <em>N</em> data points
 * (<em>x<sub>i</sub></em>, <em>y<sub>i</sub></em>) of type {@code double}
 * so that it approximates a polynomial regresison, but takes less time.
 * <p>
 * ( Due to not using matrices or using any complex calculations )
 * <p>
 * <b> NOTE: THIS IS ONLY AN APPROXIMATION OF A POLYNOMIAL REGRESSION. </b>
 * <p>
 * If you need a more accurate model, use the {@code PolynomialRegression} class.
 * 
 * @author Anish Malraj
 */
public class FastPolynomialRegression {
  public HashMap<Double, Double> m_map = new HashMap<Double, Double>();
  public TreeSet<Double> sortedKeys = new TreeSet<Double>();

  public FastPolynomialRegression(double[][] pts) {
    for (int i = 0; i < pts.length; i++) {
      m_map.put(pts[i][0], pts[i][1]);
      sortedKeys.add(pts[i][0]);
    }

    // Code to sort map - Useful, but not necessary as maps only store only one of each key
    //
    // LinkedHashMap<Double, Double> sortedMap = m_map.entrySet()
    //         .stream()
    //         .sorted(Map.Entry.comparingByKey())
    //         .collect(Collectors.toMap(
    //             Map.Entry::getKey,
    //             Map.Entry::getValue,
    //             (e1, e2) -> e1, 
    //             LinkedHashMap::new
    //         ));
    //
    // m_map = sortedMap;
  }

  /** Puts a single point into the dataset. */
  public void put(double key, double value) {
    m_map.put(key, value);
  }

  /** Gets the associated value from {@code m_map} for the given key. */
  public double get(double key) {
    if (m_map.containsKey(key)) {
      return m_map.get(key);
    }

    InterpolatingDoubleTreeMap res = new InterpolatingDoubleTreeMap();

    Double lower = sortedKeys.lower(key);
    Double upper = sortedKeys.higher(key);

    try {
      res.put(lower, m_map.get(lower));
    } catch (NullPointerException e) {
      res.put(sortedKeys.first(), m_map.get(sortedKeys.first()));
    }

    try {
      res.put(upper, m_map.get(upper));
    } catch (NullPointerException e) {
      res.put(sortedKeys.last(), m_map.get(sortedKeys.last()));
    }

    return res.get(key);
  }

  /** Clears the contents. */
  public void clear() {
    m_map.clear();
  }

  /**
   * Unit tests the {@code FastPolynomialRegression} data type.
   *
   * @param args the command-line arguments
   */
  public static void main(String[] args) {
    double[][] shotmap = new double[][] {
      {1.3123004781716114, 23.0},
      {1.7887850123341673, 18.0},
      {2.852755924182523, 7.0},
      {3.4843441022583472, 3.0},
      {4.002383309318646, 0.0}
    };

    double[] first = new double[shotmap.length];
    double[] second = new double[shotmap.length];

    for (int n = 0; n < shotmap.length; n++) {
      first[n] = shotmap[n][0];
      second[n] = shotmap[n][1];
    }

    FastPolynomialRegression wFast = new FastPolynomialRegression(
      shotmap
    );

    PolynomialRegression wPoly = new PolynomialRegression(
        first,
        second,
        5
    );

    InterpolatingDoubleTreeMap wLine = new InterpolatingDoubleTreeMap();

    for (int i = 0; i < shotmap.length; i++) {
      wLine.put(shotmap[i][0], shotmap[i][1]);
    }

    for (double testDist = first[first.length - 1]; testDist >= first[0]; testDist -= 0.5) {
      System.out.println("Distance = " + testDist);
      System.out.println("Regular: " + wLine.get(testDist));
      System.out.println("Poly: " + wPoly.predict(testDist));
      System.out.println("Fast: " + wFast.get(testDist));
      System.out.println();
    }
  }
}