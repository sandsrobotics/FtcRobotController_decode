package org.firstinspires.ftc.teamcode.parts.intake3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.State;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
import om.self.ezftc.core.part.LoopedPartImpl;

//@Config
public class IntakeTeleop3 extends LoopedPartImpl<Intake3, IntakeSettings3, ObjectUtils.Null> {
    private IntakeSettings3 settings;
    ButtonMgr buttonMgr;

    // NEW: Servo selector
    private int selectedServo = 0;   // 0, 1, or 2

    public IntakeTeleop3(Intake3 parent) {
        super(parent, "Intake teleop");
        setSettings(IntakeSettings3.makeDefault());
        buttonMgr = parent.parent.buttonMgr;
    }

    public IntakeTeleop3(Intake3 parent, IntakeSettings3 settings) {
        super(parent, "Intake teleop");
        setSettings(settings);
        buttonMgr = parent.parent.buttonMgr;
    }

    public IntakeSettings3 getSettings() {
        return settings;
    }

    public void setSettings(IntakeSettings3 settings) {
        this.settings = settings;
    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {}

    @Override
    public void onStart() {
        parent.setBaseController(() -> new IntakeControl3(false), true);
        parent.getHardware().launchMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, IntakeSettings3.spinnerPID);
    }

    @Override
    public void onRun() {
        driverControls();
    }

    @Override
    public void onStop() {
        parent.setBaseControllerToDefault(parent.isControlActive());
    }

    public void driverControls() {

        // e-stop
        if (buttonMgr.getState(2, Buttons.back, State.wasPressed) ||
                buttonMgr.getState(1, Buttons.back, State.wasPressed)) {
            parent.eStop();
        }

//        // reverse intake when 3 artifacts are in the launcher
//        if(parent.artifacts.getArtifactCount() == 3 && parent.getTargetIntakeRPM() > 0) {
//            parent.setIntakeRPM(-IntakeSettings3.intakeRPM);
//        }

        // intake/launch stuff
        if (buttonMgr.getState(1, Buttons.dpad_down, State.wasTapped)) {
            parent.setIntakeRPM(IntakeSettings3.intakeRPM);
        }

        if (buttonMgr.getState(1, Buttons.dpad_up, State.wasTapped)) {
            parent.setIntakeRPM(-IntakeSettings3.intakeRPM);
        }

        if (buttonMgr.getState(1, Buttons.dpad_down, State.wasDoubleTapped) ||
            buttonMgr.getState(1, Buttons.dpad_up, State.wasDoubleTapped)) {
            parent.setIntakeRPM(0);
        }

        if (buttonMgr.getState(1, Buttons.y, State.wasTapped)) {
            parent.setLaunchRPM(IntakeSettings3.launchRPM);
        }

        if (buttonMgr.getState(1, Buttons.y, State.wasDoubleTapped)) {
            parent.setLaunchRPM(0);
        }
        if (buttonMgr.getState(1, Buttons.right_trigger, State.isHeld)) {
            parent.computeLaunchOrder(parent.limeLight.getClassificationPattern());
        }

        if (buttonMgr.getState(1, Buttons.b, State.wasTapped)) {
            parent.stopAllIntakeTasks();
            parent.tasks.ballLaunchTask.restart();
        }

        if (buttonMgr.getState(1, Buttons.a, State.wasTapped)) {
            parent.stopAllIntakeTasks();
            parent.tasks.sameTimeBallLaunchTask.restart();
        }

        if (buttonMgr.getState(1, Buttons.x, State.wasTapped)) {
            parent.launchData = IntakeSettings3.launchPosiMap.get("blueshoot1");
            parent.tasks.moveAndLaunch.restart();
        }

        if (buttonMgr.getState(1, Buttons.left_stick_button, State.wasTapped)) {
            parent.launchData = IntakeSettings3.launchPosiMap.get("bluefartriangle");
            parent.tasks.moveAndLaunch.restart();
        }

        // left bumper: cycle servo selection
        if (buttonMgr.getState(1, Buttons.left_bumper, State.wasTapped)) {
            selectedServo = (selectedServo + 1) % 3;
        }

        // right bumper: activate selected servo
        if (buttonMgr.getState(1, Buttons.right_bumper, State.wasTapped)) {
            switch (selectedServo) {
                case 0:
                    parent.getHardware().launchServo0.setPosition(IntakeSettings3.launchServo0Launch);
                    break;
                case 1:
                    parent.getHardware().launchServo1.setPosition(IntakeSettings3.launchServo1Launch);
                    break;
                case 2:
                    parent.getHardware().launchServo2.setPosition(IntakeSettings3.launchServo2Launch);
                    break;
            }
        }

        // right bumper double tap: return selected servo to rest
        if (buttonMgr.getState(1, Buttons.right_bumper, State.wasDoubleTapped)) {
            switch (selectedServo) {
                case 0:
                    parent.getHardware().launchServo0.setPosition(IntakeSettings3.launchServo0Rest);
                    break;
                case 1:
                    parent.getHardware().launchServo1.setPosition(IntakeSettings3.launchServo1Rest);
                    break;
                case 2:
                    parent.getHardware().launchServo2.setPosition(IntakeSettings3.launchServo2Rest);
                    break;
            }
        }
    }
}
