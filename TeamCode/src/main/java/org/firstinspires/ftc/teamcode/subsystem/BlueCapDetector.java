package org.firstinspires.ftc.teamcode.subsystem;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


public class BlueCapDetector {
    private OpenCvCamera camera;
    private String cameraName;
    private HardwareMap hardwareMap;
    private ContourPipeline pipeline;
//    private Servo servo;

    public BlueCapDetector(HardwareMap hardwareMap, String cameraName){
        this.hardwareMap = hardwareMap;
        this.cameraName = cameraName;

        pipeline = new BlueCapPipeline();
    }

    public void init(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, cameraName), cameraMonitorViewId);
//        servo = hardwareMap.get(Servo.class, "camServo");
//        servo.setPosition(0.63);
//
        camera.setPipeline(pipeline = new BlueCapPipeline());
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void setPipeline(ContourPipeline pipeline) {
        camera.setPipeline(pipeline);
        this.pipeline = pipeline;
    }

    public double getX(){
        return pipeline.getX();
    }

    public void close(){
        camera.closeCameraDevice();
    }

    public void pause(){
        camera.pauseViewport();
    }
}