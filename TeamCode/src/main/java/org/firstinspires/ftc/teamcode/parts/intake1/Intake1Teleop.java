package org.firstinspires.ftc.teamcode.parts.intake1;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.intake1.settings.Intake1TeleopSettings;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.State;

import om.self.ezftc.core.part.LoopedPartImpl;

public class Intake1Teleop extends LoopedPartImpl<Intake1, Intake1TeleopSettings, ObjectUtils.Null> {
    private Intake1TeleopSettings settings;
    ButtonMgr buttonMgr;

    public Intake1Teleop(Intake1 parent) {
        super(parent, "Intake teleop");
        setSettings(Intake1TeleopSettings.makeDefault(parent.parent));
        buttonMgr = parent.parent.buttonMgr;
    }

    public Intake1Teleop(Intake1 parent, Intake1TeleopSettings settings) {
        super(parent, "Intake teleop");
        setSettings(settings);
        buttonMgr = parent.parent.buttonMgr;
    }

    public Intake1TeleopSettings getSettings() {
        return settings;
    }

    public void setSettings(Intake1TeleopSettings settings) {
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
        parent.setBaseController(() -> new Intake1Control(
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
        if (org.firstinspires.ftc.teamcode.parts.intake1.DecodeSettings.isTeleOp()) {
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
        if (!buttonMgr.getState(2, Buttons.start, State.isPressed)) {
            // Driver 2
            /*      Teleop Pre-selected Launch Tasks     */
            //        pre-selected-teleopFar
            if (buttonMgr.getState(2, Buttons.dpad_down, State.isPressed)) {
                parent.tasks.teleopFarLaunch.restart();
            }
            //       pre-selected-teleopNear
            if (buttonMgr.getState(2, Buttons.dpad_left, State.isPressed)) {
                parent.tasks.teleopNearLaunch.restart();
            }
            //       pre-selected-teleopGoal
            if (buttonMgr.getState(2, Buttons.dpad_up, State.isPressed)) {
                parent.tasks.teleopGoalLaunch.restart();
            }
            //       pre-selected-teleopThree
            if (buttonMgr.getState(2, Buttons.dpad_right, State.isPressed)) {
                parent.tasks.teleopThreeLaunch.restart();
            }

            /*      Manual FarLaunch Tasks          */
            if (buttonMgr.getState(2, Buttons.left_bumper, State.isPressed)) {
                    parent.tasks.startFarLaunch.restart();
            }
            // StopLaunch
            if (buttonMgr.getState(2, Buttons.left_trigger, State.isPressed)) {
                parent.tasks.stopLaunch.restart();
            }
            /*      Manual GoalLaunch Tasks          */
            if (buttonMgr.getState(2, Buttons.right_bumper, State.isPressed)) {
                parent.tasks.startGoalLaunch.restart();
            }
            // Manual ThreeLaunch
            if (buttonMgr.getState(2, Buttons.right_trigger, State.isPressed)) {
                parent.tasks.startThreeLaunch.restart();
            }
            /*      Launch Tasks          */
            if (buttonMgr.getState(2, Buttons.b, State.isPressed)) {
                parent.tasks.pinkServoLaunch.restart();
            }
            /*      Launch Tasks          */
            if (buttonMgr.getState(2, Buttons.y, State.isPressed)) {
                parent.tasks.blueServoLaunch.restart();
            }
            /*      Launch Tasks          */
            if (buttonMgr.getState(2, Buttons.x, State.isPressed)) {
                parent.tasks.greenServoLaunch.restart();
            }

            /*      Launch Tasks          */
            if (buttonMgr.getState(2, Buttons.a, State.isPressed)) {
                parent.tasks.pinkBlueGreenServoLaunch.restart();
            }
            /*      Launch ALL Servos Tasks          */
            if (buttonMgr.getState(2, Buttons.a, State.wasDoubleTapped)) {
                parent.tasks.computeAndLaunchInOrder.restart();
            }
        }


        // Driver 2 - start button is a "shift" key; anything below is when start is held first
        else {


        }

        // *** DRIVER 1 CONTROLS ***
        // Driver 1 - start button is a "shift" key; anything below is if start is not pushed
        if (!buttonMgr.getState(1, Buttons.start, State.isPressed)) {
            //           DRIVER 1
            //             intake Task
            if (buttonMgr.getState(1, Buttons.right_bumper, State.isPressed)) {
                parent.tasks.intakeTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.left_bumper, State.isPressed)) {
                parent.tasks.artifactIntakeStopTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.a, State.isPressed)) {
                parent.tasks.outtakeTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.x, State.isPressed)) {
                parent.tasks.viewObelisk.restart();
            }
            if (buttonMgr.getState(1, Buttons.y, State.isPressed)) {
                parent.tasks.allServoDock.restart();
            }
            if (buttonMgr.getState(1, Buttons.b, State.isPressed)) {
                parent.tasks.allServoStore.restart();
            }

                // Driver 1
            }
            // Driver 1 - start button is a "shift" key; anything below is when start is held first
            else {
                // add shifted controls here


            }
        }
    }