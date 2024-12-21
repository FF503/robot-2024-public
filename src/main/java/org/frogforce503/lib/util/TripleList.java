package org.frogforce503.lib.util;

import java.util.ArrayList;

public class TripleList<A, B, C> {
    private ArrayList<Triple<A, B, C>> list;

    public TripleList() {
        list = new ArrayList<>();
    }

    public Triple<A, B, C> getTripleByFirst(A a) {
        for (Triple<A, B, C> triple : list) {
            if (triple.getFirst().equals(a)) {
                return triple;
            }
        }
        return null;
    }

    public Triple<A, B, C> getTripleBySecond(B b) {
        for (Triple<A, B, C> triple : list) {
            if (triple.getFirst().equals(b)) {
                return triple;
            }
        }
        return null;
    }

    public Triple<A, B, C> getTripleByThird(C c) {
        for (Triple<A, B, C> triple : list) {
            if (triple.getFirst().equals(c)) {
                return triple;
            }
        }
        return null;
    }

    public ArrayList<Triple<A, B, C>> getTripleList() {
        return list;
    }

    public boolean add(Triple<A, B, C> triple) {
        list.add(triple);
        return true;
    }

    public void add(int index , Triple<A, B, C> triple) {
        list.add(index, triple);
    }

    public Triple<A, B, C> get(int index) {
        return list.get(index);
    }

    public void set(int index, Triple<A, B, C> triple) {
        list.set(index, triple);
    }

    public void remove(int index) {
        list.remove(index);
    }

    public int size() {
        return list.size();
    }

    public void clear() {
        list.clear();
    }
}
