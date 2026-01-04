package org.firstinspires.ftc.teamcode.parts.artifact;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;
import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.LoopedPartImpl;

public class Artifacts extends LoopedPartImpl<Robot, ObjectUtils.Null, ObjectUtils.Null> {
    OpenCvWebcam camera;
    private VisionPortal visionPortal;
    public ArtifactDetectionPipeline pipeline;
    public ArtifactDetectionPipeline.Artifact[] artifactList;
    public Artifacts(Robot parent) {
        super(parent, "artifacts");
    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {
        HardwareMap hardwareMap = parent.opMode.hardwareMap;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 10);
        pipeline = new ArtifactDetectionPipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    @Override
    public void onStart() {
        //pipeline.position = pipeline.getAnalysis();
    }

    @Override
    public void onRun() {
        artifactList = pipeline.getArtifactList();
        StringBuilder articolors = new StringBuilder("|");
        for( ArtifactDetectionPipeline.Artifact artifact : artifactList) {
            articolors.append(artifact.color.name()).append("|");
        }
        parent.opMode.telemetry.addData("Artifacts: ", articolors.toString());
    }

    @Override
    public void onStop() {
        if (camera.getClass() != null) {
        }
        try {
            if (camera != null) {
                camera.stopStreaming();
                camera.closeCameraDevice();
            }
        } catch (OpenCvCameraException E) {}
    }

    public ArtifactDetectionPipeline.Artifact[] getArtifactList() {
        return pipeline.getArtifactList();
    }
}
