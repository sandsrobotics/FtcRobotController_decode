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

        //todo: Read this note and fix things!
        /* **********************************************************
         *
         * IMPORTANT NOTE FROM MR. KN:
         *
         * State.isPressed should not be used when controlling tasks (or triggering anything).
         * It returns the current state of the button, not a rising or falling edge event.
         * That means it triggers the task repeatedly for the entire duration it is pressed.
         * Not good!
         *
         * Better choices would be:
         *    wasPressed;       // Rise
         *    wasReleased;      // Fall
         *    wasHeld;          // Rise + Long Hold + Fall
         *    wasTapped;        // Rise + Short Hold + Fall
         *    wasSingleTapped;  // Rise + Short Hold + Fall + Gap  [note the gap/delay at end!]
         *    wasDoubleTapped;  // Rise + Short Hold + Fall + Short Gap x2
         *
         * ***********************************************************/

        // *** DRIVER 2 CONTROLS ***
        // Driver 2 - slide control
//        parent.setUserSlidePower(-parent.parent.opMode.gamepad2.left_stick_y);
        // Driver 2 - start button is a "shift" key; anything below is if start is not pushed
        if (!buttonMgr.getState(2, Buttons.start, State.isPressed)) {
            // Driver 2
            /*      Teleop Pre-selected Launch Tasks     */
            //        pre-selected-teleopFar
            if (buttonMgr.getState(2, Buttons.dpad_down, State.wasTapped)) {
//                parent.tasks.teleopFarLaunch.restart();
                if(DecodeSettings.isAllianceRed()) {
                    parent.tasks.teleopFarRedLaunch.restart();
                } else {
                    parent.tasks.teleopFarBlueLaunch.restart();
                }
            }
            //       pre-selected-teleopNear
            if (buttonMgr.getState(2, Buttons.dpad_left, State.wasTapped)) {
//                parent.tasks.teleopNearLaunch.restart();
                if(DecodeSettings.isAllianceRed()) {
                    parent.tasks.teleopNearRedLaunch.restart();
                } else {
                    parent.tasks.teleopNearBlueLaunch.restart();
                }

            }
            //       pre-selected-teleopGoal
            if (buttonMgr.getState(2, Buttons.dpad_up, State.wasTapped)) {
//                parent.tasks.teleopGoalLaunch.restart();
                if(DecodeSettings.isAllianceRed()) {
                    parent.tasks.teleopGoalRedLaunch.restart();
                } else {
                    parent.tasks.teleopGoalBlueLaunch.restart();
                }

            }
            //       pre-selected-teleopThree
            if (buttonMgr.getState(2, Buttons.dpad_right, State.wasTapped)) {
//                parent.tasks.teleopThreeLaunch.restart();
                if(DecodeSettings.isAllianceRed()) {
                    parent.tasks.teleopThreeRedLaunch.restart();
                } else {
                    parent.tasks.teleopThreeBlueLaunch.restart();
                }
            }

            /*      Manual FarLaunch Tasks          */
            if (buttonMgr.getState(2, Buttons.left_bumper, State.wasTapped)) {
                    parent.tasks.startFarLaunch.restart();
            }
            // StopLaunch
            if (buttonMgr.getState(2, Buttons.left_trigger, State.wasTapped)) {
                parent.tasks.stopLaunch.restart();
            }
            /*      Manual GoalLaunch Tasks          */
            if (buttonMgr.getState(2, Buttons.right_bumper, State.wasTapped)) {
                parent.tasks.startGoalLaunch.restart();
            }
            // Manual ThreeLaunch
            if (buttonMgr.getState(2, Buttons.right_trigger, State.wasTapped)) {
                parent.tasks.startThreeLaunch.restart();
            }
            /*      Launch Tasks          */
            if (buttonMgr.getState(2, Buttons.b, State.wasTapped)) {
                parent.tasks.pinkServoLaunch.restart();
            }
            /*      Launch Tasks          */
            if (buttonMgr.getState(2, Buttons.y, State.wasTapped)) {
                parent.tasks.blueServoLaunch.restart();
            }
            /*      Launch Tasks          */
            if (buttonMgr.getState(2, Buttons.x, State.wasTapped)) {
                parent.tasks.greenServoLaunch.restart();
            }

            /*      Launch Tasks          */
            if (buttonMgr.getState(2, Buttons.a, State.wasTapped)) {
//                parent.tasks.pinkBlueGreenServoLaunch.restart();
                  parent.tasks.allServoLaunch.restart();

            }
            /*      Launch ALL Servos Tasks          */
            if (buttonMgr.getState(2, Buttons.a, State.wasHeld)) {
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
            if (buttonMgr.getState(1, Buttons.right_bumper, State.wasTapped)) {
                parent.tasks.intakeTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.left_bumper, State.wasTapped)) {
                parent.tasks.artifactIntakeStopTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.a, State.wasTapped)) {
                parent.tasks.outtakeTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.x, State.wasTapped)) {
                parent.tasks.viewObelisk.restart();
            }
            if (buttonMgr.getState(1, Buttons.y, State.wasTapped)) {
                parent.tasks.allServoDock.restart();
            }
            if (buttonMgr.getState(1, Buttons.b, State.wasTapped)) {
                parent.tasks.allServoStore.restart();
            }

                // Driver 1
            }
            // Driver 1 - start button is a "shift" key; anything below is when start is held first
        else {
            // add shifted controls here

            // LK new test
            if (DecodeSettings.lkTestMode1) {
                if (buttonMgr.getState(1, Buttons.dpad_up, State.wasTapped)) {
                    parent.headingSolver.startSolver();
                }
                if (buttonMgr.getState(1, Buttons.dpad_left, State.wasTapped)) {
                    parent.headingSolver.setNewTarget(DecodeSettings.targetBlue, true);
                }
                if (buttonMgr.getState(1, Buttons.dpad_right, State.wasTapped)) {
                    parent.headingSolver.setNewTarget(DecodeSettings.targetRed, true);
                }
                if (buttonMgr.getState(1, Buttons.dpad_down, State.wasTapped)) {
                    parent.headingSolver.stopSolver();
                }
            }
            if (DecodeSettings.lkTestMode2) {
                if (buttonMgr.getState(1, Buttons.dpad_up, State.wasTapped)) {
                    parent.headerAimer.setAutoAim(true);
                }
                if (buttonMgr.getState(1, Buttons.dpad_left, State.wasTapped)) {
                    parent.headerAimer.setTarget(DecodeSettings.targetBlue);
                }
                if (buttonMgr.getState(1, Buttons.dpad_right, State.wasTapped)) {
                    parent.headerAimer.setTarget(DecodeSettings.targetRed);
                }
                if (buttonMgr.getState(1, Buttons.dpad_down, State.wasTapped)) {
                    parent.headerAimer.setAutoAim(false);
                }
            }
            if (DecodeSettings.lkTestMode1 || DecodeSettings.lkTestMode2) {
                if (buttonMgr.getState(1, Buttons.y, State.wasDoubleTapped)) {
                    parent.limeLight.applyTransform();
                    parent.positionTracker.setOverrideTransform(parent.limeLight.llSavedTransform);
                }
            }
        }
    }
}