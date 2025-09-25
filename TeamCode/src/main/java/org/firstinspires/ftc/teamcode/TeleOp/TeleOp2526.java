package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TeleOp2526_FieldCentric")
public class TeleOp2526 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        float speed = 0.75f;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Get the current pose (position + heading)
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Joystick inputs (translation)
            double forward = -gamepad1.left_stick_y * speed; // forward/back
            double strafe  = -gamepad1.left_stick_x * speed; // strafe left/right
            double rotate  = -gamepad1.right_stick_x * speed; // rotation

            // Create input vector
            Vector2d input = new Vector2d(strafe, forward);

            // Rotate by inverse of heading for field-centric control
            input = input.rotated(-poseEstimate.getHeading());

            // Apply movement
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            rotate
                    )
            );

            // Update Road Runner localization
            drive.update();

            // Telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading (rad)", poseEstimate.getHeading());
            telemetry.addData("heading (deg)", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
        }
    }
}
