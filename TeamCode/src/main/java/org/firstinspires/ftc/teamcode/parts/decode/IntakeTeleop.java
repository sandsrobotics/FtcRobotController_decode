package org.firstinspires.ftc.teamcode.parts.decode;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.decode.settings.IntakeTeleopSettings;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.State;

import om.self.ezftc.core.part.LoopedPartImpl;

public class IntakeTeleop extends LoopedPartImpl<Intake, IntakeTeleopSettings, ObjectUtils.Null> {
    private IntakeTeleopSettings settings;
    ButtonMgr buttonMgr;

    public IntakeTeleop(Intake parent) {
        super(parent, "Intake teleop");
        setSettings(IntakeTeleopSettings.makeDefault(parent.parent));
        buttonMgr = parent.parent.buttonMgr;
    }

    public IntakeTeleop(Intake parent, IntakeTeleopSettings settings) {
        super(parent, "Intake teleop");
        setSettings(settings);
        buttonMgr = parent.parent.buttonMgr;
    }

    public IntakeTeleopSettings getSettings() {
        return settings;
    }

    public void setSettings(IntakeTeleopSettings settings) {
        this.settings = settings;
    }

    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onInit() {

    }

    @Override
    public void onStart() {
        parent.setBaseController(() -> new IntakeControl(
                0
        ), true);
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
        // e-stop, either driver
        if (buttonMgr.getState(2, Buttons.back, State.wasPressed) ||
                buttonMgr.getState(1, Buttons.back, State.wasPressed)) {
            parent.eStop();
        }

        //in telelop, disable positiontracker if Driver1 uses controls
        if (org.firstinspires.ftc.teamcode.parts.decode.DecodeSettings.isTeleOp()) {
            if (parent.parent.opMode.gamepad1.right_trigger +
                    parent.parent.opMode.gamepad1.left_trigger +
                    parent.parent.opMode.gamepad1.left_stick_x +
                    parent.parent.opMode.gamepad1.left_stick_y +
                    parent.parent.opMode.gamepad1.right_stick_x +
                    parent.parent.opMode.gamepad1.right_stick_y != 0) {
//                parent.positionSolver.stopSolver();
            }
        }

        // *** DRIVER 2 CONTROLS ***
        // Driver 2 - slide control
//        parent.setUserSlidePower(-parent.parent.opMode.gamepad2.left_stick_y);
        // Driver 2 - start button is a "shift" key; anything below is if start is not pushed
        /*           Intake Task Initiater            */
        if (!buttonMgr.getState(2, Buttons.start, State.isPressed)) {
            // Driver 2

        }


        // Driver 2 - start button is a "shift" key; anything below is when start is held first
        else {

            if (buttonMgr.getState(2, Buttons.y, State.wasTapped)) {
                org.firstinspires.ftc.teamcode.parts.decode.DecodeSettings.isYellowGood = !org.firstinspires.ftc.teamcode.parts.decode.DecodeSettings.isYellowGood;
            }
            // test out-take speed
            if (buttonMgr.getState(2, Buttons.left_bumper, State.wasTapped)) {
                parent.testSpinnerOut += parent.sIncrement;
                parent.testSpinnerOut = Math.max(0, Math.min(parent.testSpinnerOut, 0.5));
            }
            if (buttonMgr.getState(2, Buttons.right_bumper, State.wasTapped)) {
                parent.testSpinnerOut -= parent.sIncrement;
                parent.testSpinnerOut = Math.max(0, Math.min(parent.testSpinnerOut, 0.5));
            }
        }

        // *** DRIVER 1 CONTROLS ***
        // Driver 1 - start button is a "shift" key; anything below is if start is not pushed
        if (!buttonMgr.getState(1, Buttons.start, State.isPressed)) {
            /*          Intake Task Initiator            */
            /*             intake suppliers          */
            if (buttonMgr.getState(2, Buttons.right_bumper, State.wasTapped)) {
                if (parent.getHardware().intakeServo.isSetPosition(parent.getSettings().servoIntakeOut)) {
                    parent.setServoIntake(parent.getSettings().servoIntakeStop);

                } else {
                    parent.setServoIntake(parent.getSettings().servoIntakeOut);
                    parent.tasks.artifactExtakeTask.restart();
                }
            }
            if (buttonMgr.getState(2, Buttons.left_bumper, State.wasTapped)) {
                if (parent.getHardware().intakeServo.isSetPosition(parent.getSettings().servoIntakeIn)) {
                    parent.setServoIntake(parent.getSettings().servoIntakeStop);
                } else {
                    parent.setServoIntake(parent.getSettings().servoIntakeIn);
                    parent.tasks.artifactIntakeTask.restart();
                }

                // Driver 1
            }
            // Driver 1 - start button is a "shift" key; anything below is when start is held first
            else {
                // add shifted controls here


            }
        }
    }}