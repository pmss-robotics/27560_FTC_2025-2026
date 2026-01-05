package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.util.StateTransfer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

@Config // Makes variables changeable on the FTC dashboard
public class TurretVisionSubsystem extends SubsystemBase {

    public static int EXPOSURE_MS = 6;
    public static int GAIN = 190;

    private AprilTagProcessor goalAprilTag;
    private static AprilTagLibrary decodeGoal;
    private VisionPortal visionPortal;
    private Telemetry telemetry;
    private double red, blue;
    public TurretVisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry, boolean debug) throws InterruptedException {
        this.telemetry = telemetry;

        final CameraStreamProcessor dashboard = new CameraStreamProcessor();
        
        goalAprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(debug)
                .setDrawCubeProjection(debug)
                .setDrawTagOutline(debug)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(getDecodeGoalLibrary())
                .build();


        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setAutoStopLiveView(true)
                .enableLiveView(debug)
                .addProcessors(dashboard, goalAprilTag)
                .build();

        visionPortal.setProcessorEnabled(goalAprilTag, false);
        
        ElapsedTime elapsedTime = new ElapsedTime();
        VisionPortal.CameraState cameraState = visionPortal.getCameraState();

        while(elapsedTime.seconds() < 20 && cameraState != VisionPortal.CameraState.STREAMING){
            cameraState = visionPortal.getCameraState();
        }
        telemetry.log().add("Time from build to streaming (Webcam2) " + elapsedTime.milliseconds() + "ms");
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(GAIN);
            sleep(20);
        }

        FtcDashboard.getInstance().startCameraStream(dashboard, 15);
    }

    @Override
    public void periodic() {
        telemetryAprilTag();
        telemetry.update();
    }

    public void enableDetection(boolean enabled) {
        visionPortal.setProcessorEnabled(goalAprilTag, enabled);
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = goalAprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format(Locale.CANADA, "\n==== (ID %d) %s", detection.id, detection.metadata.name));
            } else {
                telemetry.addLine(String.format(Locale.CANADA, "\n==== (ID %d) Unknown", detection.id));
            }

        }
    }

    public double update() {
        List<AprilTagDetection> currentDetections = goalAprilTag.getDetections();
        double red = 0, blue = 0;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                switch (detection.id) {
                    case 20: blue = detection.ftcPose.bearing;
                    case 24: red = detection.ftcPose.bearing;
                }
            }
        }

        try {
            switch (StateTransfer.alliance) {
                case Red: return red;
                case Blue: return blue;
            }
        } catch (NullPointerException e) {
            telemetry.addLine("Please set the alliance - i think its null for some reason");
            telemetry.update();
        }

        //wwaj;eoifja;lweiufoiwue;rfu8w4yehf;uih;ewisdjflkwejf;lakwejsd;lj


        //TODO: extra correction if you see other goal april tag

        // Fallback
        return Double.NaN;
    }

    private AprilTagLibrary getDecodeGoalLibrary() {
        if (decodeGoal == null) {
            AprilTagLibrary decodeLibrary = AprilTagGameDatabase.getDecodeTagLibrary();
            decodeGoal = new AprilTagLibrary.Builder()
                    .addTag(decodeLibrary.lookupTag(20))
                    .addTag(decodeLibrary.lookupTag(24))
                    .build();
        }
        return decodeGoal;
    }

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }
}