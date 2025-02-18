package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

public class Positions {
    public double left;
    public double right;
    public double avg;

    public Positions(double left, double right) {
        this.left  = left;
        this.right = right;
        this.avg   = (right + left) / 2;
    }

    @NonNull
    public String toString () {
        return (left + "," + right);
    }
}