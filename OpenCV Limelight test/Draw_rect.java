package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.Collections;

import org.openftc.easyopencv.OpenCvPipeline;

public class Draw_rect extends OpenCvPipeline {

	public Scalar lowerHSV = new Scalar(6.0, 142.0, 0.0, 0.0);
	public Scalar upperHSV = new Scalar(28.0, 239.0, 255.0, 0.0);
	// public Scalar lowerHSV = new Scalar(0, 0, 0, 0);
	// public Scalar upperHSV = new Scalar(255, 255, 255, 0);
	private Mat hsvMat = new Mat();
	private Mat hsvBinaryMat = new Mat();

	private ArrayList<MatOfPoint> contours = new ArrayList<>();
	private Mat hierarchy = new Mat();

	private MatOfPoint2f contours2f = new MatOfPoint2f();
	private ArrayList<RotatedRect> contoursRects = new ArrayList<>();

	private RotatedRect biggestRect = null;

	public Scalar lineColor = new Scalar(0.0, 255.0, 0.0, 0.0);
	public int lineThickness = 3;

	private Mat inputRects = new Mat();

	public RotatedRect lastRes;

	@Override
	public Mat processFrame(Mat input) {
		// for(MatOfPoint points : contours) {
		// 	contoursRects.add(Imgproc.boundingRotatedRect(points));
		// }

		// this.biggestRect = null;
		// for(RotatedRect rect : contoursRects) {
		// 	if(rect != null) {
		// 		if((biggestRect == null) || (rect.size.area() > biggestRect.size.area() && rect.size.area() > 0.2)) {
		// 			this.biggestRect = rect;
		// 		}
		// 	}
		// }

		// input.copyTo(inputRects);
		// if(biggestRect != null) {
		// 	Imgproc.rectangle(inputRects, biggestRect, lineColor, lineThickness);
		// }

		// clearTargets();

		// addTarget("Sample", biggestRect);
		// // telemetry.addData("Sample", biggestRect.tl());
		// lastRes = biggestRect;
		// return inputRects;

		Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
		Core.inRange(hsvMat, lowerHSV, upperHSV, hsvBinaryMat);

		contours.clear();
		hierarchy.release();
		Imgproc.findContours(hsvBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

		contoursRects.clear();
		for(MatOfPoint points : contours) {
			contours2f.release();
			points.convertTo(contours2f, CvType.CV_32F);

			contoursRects.add(Imgproc.minAreaRect(contours2f));
		}

		this.biggestRect = null;
		for(RotatedRect rect : contoursRects) {
			if(rect != null) {
				
				if((biggestRect == null) || (rect.size.area() > biggestRect.size.area() && rect.size.area() > 0.2)) {
					this.biggestRect = rect;
				}
			}
		}

		if (biggestRect != null) {
			Point[] rectPoints = new Point[4];
			biggestRect.points(rectPoints);
			MatOfPoint matOfPoint = new MatOfPoint(rectPoints);

			Imgproc.polylines(input, Collections.singletonList(matOfPoint), true, lineColor, lineThickness);

		}

		input.copyTo(inputRects);
		
		lastRes = biggestRect;
		return inputRects;

		// Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2GRAY);
        // return hsvMat;
	}

	public RotatedRect getLatestRes () {
		return lastRes;
	}

}