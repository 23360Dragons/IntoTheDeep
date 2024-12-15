package org.firstinspires.ftc.teamcode.utils;

public class LLPipeline {
    public int num;

    public LLPipeline (int pipeline) {
        this.num = pipeline;
    }

    public String getName() {
        String name;
        switch (num) {
            case 0: name   = "BLUE";    break;
            case 1: name   = "RED";     break;
            case 2: name   = "YELLOW";  break;
            default : name = "Unknown"; break;
        };
        return name;
    }
}
