package org.firstinspires.ftc.teamcode.parts.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.artifact.ArtifactDetectionPipeline.ArtifactColor;
import java.util.List;
import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.LoopedPartImpl;

public class LimeLight extends LoopedPartImpl<Robot, ObjectUtils.Null, ObjectUtils.Null> {
    private Limelight3A limelight;
    public LimeLight(Robot parent) {
        super(parent, "limelight");
    }
    private ArtifactColor[] classificationPattern = new ArtifactColor[]{
            ArtifactColor.NONE,
            ArtifactColor.NONE,
            ArtifactColor.NONE
    };
    // Limelight data
    public double tx = 0;  // Horizontal offset from crosshair
    public double ty = 0;  // Vertical offset from crosshair
    public double ta = 0;  // Target area (0-100% of image)
    public boolean tv = false;  // Valid target flag
    // ===== AprilTag latch state =====
    private Integer lastSeenAprilTagId = null;
    private boolean hasLatchedAprilTag = false;

    private void updatePatternFromTag(int id) {
        if (id == 21) {
            classificationPattern = new ArtifactColor[]{
                    ArtifactColor.GREEN,
                    ArtifactColor.PURPLE,
                    ArtifactColor.PURPLE
            };
        } else if (id == 22) {
            classificationPattern = new ArtifactColor[]{
                    ArtifactColor.PURPLE,
                    ArtifactColor.GREEN,
                    ArtifactColor.PURPLE
            };
        } else if (id == 23) {
            classificationPattern = new ArtifactColor[]{
                    ArtifactColor.PURPLE,
                    ArtifactColor.PURPLE,
                    ArtifactColor.GREEN
            };
        }
    }

    @Override
    public void onRun() {
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
//            Pose3D botpose = result.getBotpose();
//            parent.opMode.telemetry.addData("tx", result.getTx());
//            parent.opMode.telemetry.addData("txnc", result.getTxNC());
//            parent.opMode.telemetry.addData("ty", result.getTy());
//            parent.opMode.telemetry.addData("tync", result.getTyNC());
//            parent.opMode.telemetry.addData("Botpose", botpose.toString());
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            boolean sawTrackingTag = false;

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                int id = fr.getFiducialId();
                parent.opMode.telemetry.addData("April Tag", "ID: %d", id);

                // Latched tags
                if (id == 21 || id == 22 || id == 23) {
                    if (!hasLatchedAprilTag || id != lastSeenAprilTagId) {
                        updatePatternFromTag(id);
                        lastSeenAprilTagId = id;
                        hasLatchedAprilTag = true;
                    }
                }

                // Non-Latched tags.
                if (id == 20 || id == 24) {
                    tv = true;
                    tx = result.getTx();
                    ty = result.getTy();
                    ta = result.getTa();
                    sawTrackingTag = true;
                }
            }

            if (!sawTrackingTag) {
                tv = false;
            }

            // Access color results
//            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//            for (LLResultTypes.ColorResult cr : colorResults) {
//                parent.opMode.telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
//            }
        } else {
            parent.opMode.telemetry.addData("Limelight", "No data available");
        }
    }

    public ArtifactColor[] getClassificationPattern() {
        return classificationPattern;
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
