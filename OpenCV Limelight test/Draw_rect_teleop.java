package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name="draw rect")
public class Draw_rect_teleop extends OpMode {
    OpenCvCamera camera;
    Draw_rect draw_rect;

    @Override
    public void init () {
        draw_rect = new Draw_rect();
        telemetry.addLine("init");

        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);

                camera.setPipeline(draw_rect);

                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                * This will be called if the camera could not be opened
                */
            }
        });
        
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
    }

    @Override
    public void loop () {
        telemetry.addData("loop", 1);

        camera.showFpsMeterOnViewport(false);
        
        telemetry.addData("angle",draw_rect.getLatestRes().angle);
        telemetry.addData("area", draw_rect.getLatestRes().size.area());
        telemetry.update();

    }
}
