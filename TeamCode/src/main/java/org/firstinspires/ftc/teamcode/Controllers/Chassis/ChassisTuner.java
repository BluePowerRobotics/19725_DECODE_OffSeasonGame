package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="ChassisTuner", group="Tests")

public class ChassisTuner extends LinearOpMode {
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;
    @Override
    public void runOpMode() throws InterruptedException {
        fl=hardwareMap.get(DcMotorEx.class, "fL");
        fr=hardwareMap.get(DcMotorEx.class, "fR");
        bl=hardwareMap.get(DcMotorEx.class, "bL");
        br=hardwareMap.get(DcMotorEx.class, "bR");
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        boolean moving = false;
        final int ENCODER_DELTA = 2000; // move this many encoder ticks from current position
        final double POWER = 0.6; // power while moving

        while(opModeIsActive()){
            if(gamepad1.aWasPressed() && !moving){
                // compute absolute target positions relative to current positions
                int flTarget = fl.getCurrentPosition() + ENCODER_DELTA;
                int frTarget = fr.getCurrentPosition() + ENCODER_DELTA;
                int blTarget = bl.getCurrentPosition() + ENCODER_DELTA;
                int brTarget = br.getCurrentPosition() + ENCODER_DELTA;

                // set targets and go
                fl.setTargetPosition(flTarget);
                fr.setTargetPosition(frTarget);
                bl.setTargetPosition(blTarget);
                br.setTargetPosition(brTarget);

                fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                fl.setPower(POWER);
                fr.setPower(POWER);
                bl.setPower(POWER);
                br.setPower(POWER);

                moving = true;
            }

            // while moving, show telemetry and wait until all motors finish
            if(moving){
                telemetry.addData("FL pos", fl.getCurrentPosition());
                telemetry.addData("FR pos", fr.getCurrentPosition());
                telemetry.addData("BL pos", bl.getCurrentPosition());
                telemetry.addData("BR pos", br.getCurrentPosition());
                telemetry.addData("FL target", fl.getTargetPosition());
                telemetry.addData("FR target", fr.getTargetPosition());
                telemetry.addData("all busy", (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()));
                telemetry.update();

                // if none are busy, stop
                if(!(fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())){
                    fl.setPower(0);
                    fr.setPower(0);
                    bl.setPower(0);
                    br.setPower(0);

                    // return to encoder run mode so we can read positions / run again
                    fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    moving = false;
                    telemetry.addLine("Reached target");
                    telemetry.update();
                }
            }

            // small idle to avoid busy spin
            sleep(20);
        }


    }
}
