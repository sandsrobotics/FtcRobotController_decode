package org.firstinspires.ftc.teamcode.parts.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;

import java.util.List;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.LoopedPartImpl;

public class LimeLight extends LoopedPartImpl<Robot, ObjectUtils.Null, ObjectUtils.Null> {
    private Limelight3A limelight;
    public LimeLight(Robot parent) {
        super(parent, "limelight");
    }

    // classificationId Defaults to 21.
    // Valid values are 21(GPP), 22(PGP), 23(PPG).
    private static Integer classificationId = 21;

    @Override
    public void onRun() {
        LLStatus status = limelight.getStatus();
        parent.opMode.telemetry.addData("Name", "%s", status.getName());
//        parent.opMode.telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                status.getTemp(), status.getCpu(),(int)status.getFps());
//        parent.opMode.telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                status.getPipelineIndex(), status.getPipelineType());
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            parent.opMode.telemetry.addData("tx", result.getTx());
            parent.opMode.telemetry.addData("txnc", result.getTxNC());
            parent.opMode.telemetry.addData("ty", result.getTy());
            parent.opMode.telemetry.addData("tync", result.getTyNC());
            parent.opMode.telemetry.addData("Botpose", botpose.toString());
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                parent.opMode.telemetry.addData("April Tag", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                int id = fr.getFiducialId();
                if (id == 21 || id == 22 || id == 23) {
                    classificationId = id;
                }
            }
            // Access color results
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
                parent.opMode.telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
            }
        } else {
            parent.opMode.telemetry.addData("Limelight", "No data available");
        }
    }

    public Integer getClassificationId() {
        return classificationId;
    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {
        limelight = parent.opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        parent.opMode.telemetry.setMsTransmissionInterval(11);
        limelight.start();
    }

    @Override
    public void onStart() {}

    @Override
    public void onStop() {
        limelight.stop();
    }
}
