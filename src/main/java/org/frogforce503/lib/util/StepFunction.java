package org.frogforce503.lib.util;

import java.util.TreeMap;

public class StepFunction {
    private TreeMap<Double, Double> map = new TreeMap<>();

    public void put(double x, double y) {
        map.put(x, y);
    }

    public double get(double lookupX) {
        return get_FLOOR(lookupX);
    }
    
    public double get_FLOOR(double lookupX) {
        var floor = map.floorEntry(lookupX);
        if (floor == null) {
            return map.get(map.firstKey());
        }
        return floor.getValue();
    }

    public double get_CEIL(double lookupX) {
        var floor = map.ceilingEntry(lookupX);
        if (floor == null) {
            return map.get(map.firstKey());
        }
        return floor.getValue();
    }
}
