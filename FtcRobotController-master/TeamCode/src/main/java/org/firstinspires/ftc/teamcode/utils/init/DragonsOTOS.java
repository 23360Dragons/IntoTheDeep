package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorSparkFunOTOS;

public class DragonsOTOS {
    public static SensorSparkFunOTOS initialize (HardwareMap hardwareMap) {
        SensorSparkFunOTOS otos = hardwareMap.get(SensorSparkFunOTOS.class, "");
        return new SensorSparkFunOTOS();
    }
}
