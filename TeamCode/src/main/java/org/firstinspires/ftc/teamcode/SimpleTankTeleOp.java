package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Simple Tank TeleOp", group = "TeleOp")
public class SimpleTankTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        // The names "left" and "right" must match your configuration in the Control Hub.
        // The name "feeder" must also be added to your configuration for the 3rd motor.
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "left");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, "right");
        DcMotor feederMotor = hardwareMap.get(DcMotor.class, "feeder");

        // Motor directions.
        // Depending on how your motors are mounted, you might need to change these.
        // Standard Tank Drive usually requires one side to be reversed.
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        feederMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset encoders to 0 for the demonstration of reading them
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        feederMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to RUN_WITHOUT_ENCODER so we can control power directly,
        // but the encoders will still count and be readable.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feederMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Variables for toggle logic
        boolean isFeederOn = false;
        boolean lastL1State = false;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Left Stick Y: Drive | Right Stick X: Turn | L1: Toggle Feeder");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Arcade Drive Control
            // Left Stick Y: Forward/Backward (Note: Y is negative when pushed up, so we negate it)
            // Right Stick X: Rotate Left/Right
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Combine drive and turn for blended motion
            double leftPower = drive + turn;
            double rightPower = drive - turn;

            // Normalize powers if they exceed +/- 1.0 to maintain steering ratio
            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // 2. Feeder Control (Toggle with L1)
            boolean currentL1State = gamepad1.left_bumper;
            if (currentL1State && !lastL1State) {
                // Button just pressed: toggle state
                isFeederOn = !isFeederOn;
            }
            lastL1State = currentL1State;

            if (isFeederOn) {
                feederMotor.setPower(1.0); // Full speed
            } else {
                feederMotor.setPower(0.0); // Stop
            }

            // 3. Telemetry (Reading Encoders)
            // This shows how to read the built-in encoder sensors from the REV Core Hex motors.
            telemetry.addData("Status", "Running");
            telemetry.addData("Feeder State", isFeederOn ? "ON" : "OFF");
            telemetry.addData("--- Encoder Values ---", "");
            telemetry.addData("Left Motor", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor", rightMotor.getCurrentPosition());
            telemetry.addData("Feeder Motor", feederMotor.getCurrentPosition());

            telemetry.update();
        }
    }
}
