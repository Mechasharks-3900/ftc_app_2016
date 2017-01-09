package org.firstinspires.ftc.teamcode;

/**
 * Created by holDM on 9/9/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.lang.Math;

@TeleOp(name="MultiDrive_JZ", group="Anydir robot")
public class MultiDrive extends OpMode {
    DcMotor motor_ne;
    DcMotor motor_se;
    DcMotor motor_sw;
    DcMotor motor_nw;

    DcMotor motor_launcher, motor_collector, motor_balltread;


    @Override
    public void init() {
        motor_ne = hardwareMap.dcMotor.get("motor_ne");
        motor_se = hardwareMap.dcMotor.get("motor_se");
        motor_sw = hardwareMap.dcMotor.get("motor_sw");
        motor_nw = hardwareMap.dcMotor.get("motor_nw");

        motor_launcher = hardwareMap.dcMotor.get("motor_launcher");
        motor_collector = hardwareMap.dcMotor.get("motor_collector");
        motor_balltread = hardwareMap.dcMotor.get("motor_balltread");

        motor_se.setDirection(DcMotor.Direction.REVERSE);
        motor_sw.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double tread;

        double left_x = gamepad1.left_stick_x;
        double left_y = gamepad1.left_stick_y;

        double dist = Math.sqrt(Math.pow(left_x, 2.0) + Math.pow(left_y, 2.0));
        dist = Math.max(-1.0, Math.min(1.0, dist)); // Clamp dist to [-1.0, 1.0]
        double angle = Math.atan(left_y / left_x);
        if (left_x == 0.0) {
            if (left_y < 0.0) {
                angle = -Math.PI / 2;
            } else {
                angle = Math.PI / 2;
            }
        }
        if (left_x < 0.0) {
            if (left_y <= 0.0) {
                angle -= Math.PI;
            } else {
                angle += Math.PI;
            }
        }


        telemetry.addData("joy", left_x + " " + left_y);
        telemetry.addData("angle", (angle / Math.PI) * 180);

        double angle_cos = Math.cos(angle - Math.PI / 4.0);
        double angle_sin = Math.sin(angle - Math.PI / 4.0);
        motor_ne.setPower(motor_clamp( angle_cos * dist + gamepad1.right_stick_x));
        motor_se.setPower(motor_clamp( angle_sin * dist + gamepad1.right_stick_x));
        motor_sw.setPower(motor_clamp(-angle_cos * dist + gamepad1.right_stick_x));
        motor_nw.setPower(motor_clamp(-angle_sin * dist + gamepad1.right_stick_x));

        telemetry.addData("motor_ne", motor_ne.getPower());
        telemetry.addData("motor_se", motor_se.getPower());
        telemetry.addData("motor_sw", motor_sw.getPower());
        telemetry.addData("motor_nw", motor_nw.getPower());

        if(gamepad1.left_bumper){tread = 1;} else {tread = 0;}
        if(gamepad1.right_bumper){motor_launcher.setDirection(DcMotor.Direction.FORWARD);} else {motor_launcher.setDirection(DcMotor.Direction.REVERSE);}

        telemetry.addData("motor_launcher", gamepad1.right_trigger);
        motor_balltread.setPower(-tread);
        motor_collector.setPower(-gamepad1.left_trigger);
        motor_launcher.setPower(gamepad1.right_trigger);
    }

    private double motor_clamp(double n) {
        return Math.max(-1.0, Math.min(1.0, n));
    }
}
