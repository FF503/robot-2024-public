package org.frogforce503.lib.util;

public class Triple<A, B, C> {
    private final A first;
    private final B second;
    private final C third;

    public Triple(A a, B b, C c) {
        first = a;
        second = b;
        third = c;
    }

    public A getFirst() {
        return first;
    }

    public B getSecond() {
        return second;
    }

    public C getThird() {
        return third;
    }

    public static <A, B, C> Triple<A, B, C> of(A a, B b, C c) {
        return new Triple<>(a, b, c);
    }

}