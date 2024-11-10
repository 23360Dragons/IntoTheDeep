package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

public class Positions {
    public int left;
    public int right;

    public Positions(int left, int right) {
        this.left = left;
        this.right = right;
    }

    @NonNull
    public String toString () {
        return (String.valueOf(left) + "," + String.valueOf(right));
    }
}
