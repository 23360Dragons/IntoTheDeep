package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

public class Positions {
    public double left;
    public double right;

    public Positions(double left, double right) {
        this.left = left;
        this.right = right;
    }

    @NonNull
    public String toString () {
        return (String.valueOf(left) + "," + String.valueOf(right));
    }
}