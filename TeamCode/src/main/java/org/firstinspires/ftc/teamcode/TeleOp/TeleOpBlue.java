//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import android.util.Size;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.SortOrder;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Autonomous.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
//import org.firstinspires.ftc.vision.opencv.ColorRange;
//import org.firstinspires.ftc.vision.opencv.ImageRegion;
//import org.opencv.core.RotatedRect;
//
//import java.util.List;
//
//@TeleOp(name = "TeleOp2425")
//public class TeleOpBlue extends LinearOpMode {
//
//    double speedCoefficient = 0.75;
//    double turningCoefficient = 0.5;
//
//    DcMotor horizontalSlide;
//    DcMotor verticalSlide;
//
//    Servo armRight;
//    Servo armLeft;
//
//    public double armPosition;
//    public static double armServoDown = 0.56;
//    public static double armServoUp = 0.05;
//
//    Servo basketServo;
//    public static double basketServoOpen = 0.735;
//    public static double basketServoClosed = 1;
//
//    Servo clawServo;
//    public static double clawServoOpen = 0.64;
//    public static double clawServoClosed = 1;
//
//
//    Servo clawRotate;
//    public static double clawRotateDropOff = 1.0/3.0;
//
//    Servo clawPivot;
//    public static double clawPivotUp = 0.67;
//    public static double clawPivotDown = 0.15;
//
//    double sampleAngle;
//    double angle;
//
//    private SampleMecanumDrive drive;
//
//    private boolean isForward = false;
//    private boolean spun = false;
//
//
//    ElapsedTime timer = new ElapsedTime();
//
//    private enum PickupStage {
//        PICKUP_START,
//        MOVE_SLIDE,
//        ARM_DOWN,
//        CLOSE_CLAW,
//        BACK_TO_START,
//        ARM_UP,
//        TRANSFER
//    };
//    PickupStage pickupStage = PickupStage.PICKUP_START;
//
//    ColorBlobLocatorProcessor.Blob b;
//    ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
//            .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
//            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
//            .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
//            .setDrawContours(true)                        // Show contours on the Stream Preview
//            .setBlurSize(5)                               // Smooth the transitions between different colors in image
//            .build();
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        //Initialize motors
//        verticalSlide = hardwareMap.get(DcMotor.class, "verticalSlide");
//        horizontalSlide = hardwareMap.get(DcMotor.class, "horizontalSlide");
//
//        armLeft = hardwareMap.get(Servo.class, "armLeft");
//        armRight = hardwareMap.get(Servo.class, "armRight");
//        basketServo = hardwareMap.get(Servo.class,"basketServo");
//        clawServo = hardwareMap.get(Servo.class,"clawServo");
//        clawRotate = hardwareMap.get(Servo.class, "clawRotate");
//        clawPivot = hardwareMap.get(Servo.class, "clawPivot");
//
//        //TODO: Uncomment when out of competition
////        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
//        verticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        armPosition = armServoDown - 0.02;
//        moveArmServo();
//
//        //for field centric
//        drive = new SampleMecanumDrive(hardwareMap);
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        VisionPortal portal = new VisionPortal.Builder()
//                .addProcessor(colorLocator)
//                .setCameraResolution(new Size(320, 240))
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .build();
//
//        waitForStart();
//        if(opModeIsActive()){
//            while(opModeIsActive()){
//
//                //orienting the robot for field centric
//                if(!isForward){
//                    orienting(gamepad1);
//                    verticalSlideManual(gamepad2);
//                    telemetry.update();
//                    if(gamepad1.y){
//                        drive = new SampleMecanumDrive(hardwareMap);
//                        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                        isForward = true;
//                    }
//                }
//
//                else{
//                    fieldCentricDrive(gamepad1);
//                    moveArmServo();
//                    verticalSlideManual(gamepad2);
//                    pickup(gamepad1);
//                    dropSample(gamepad2);
//
//                    telemetry.addData("horizontal slide: ", horizontalSlide.getCurrentPosition());
//
//                    telemetry.update();
//                }
//            }
//        }
//    }
//
//
//    public void fieldCentricDrive(Gamepad gamepad){
//
//        //speed boost when holding right bumper button
//        if(gamepad.right_bumper){
//            speedCoefficient = 0.9;
//            turningCoefficient = 0.9;
//        }
//        else{
//            speedCoefficient = 0.7;
//            turningCoefficient = 0.5;
//        }
//
//        // Read pose
//        Pose2d poseEstimate = drive.getPoseEstimate();
//
//
//        // Create a vector from the gamepad x/y inputs
//        // Then, rotate that vector by the inverse of that heading
//        Vector2d input = new Vector2d(
//                -gamepad.left_stick_y * speedCoefficient,
//                -gamepad.left_stick_x * speedCoefficient
//        ).rotated(-poseEstimate.getHeading());
//
//        // Pass in the rotated input + right stick value for rotation
//        // Rotation is not part of the rotated input thus must be passed in separately
//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        input.getX(),
//                        input.getY(),
//                        -gamepad.right_stick_x * turningCoefficient
//                )
//        );
//
//        // Update everything. Odometry. Etc.
//        drive.update();
//    }
//
//
//    public void orienting(Gamepad gamepad){
//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        -gamepad.left_stick_y * 0.5,
//                        -gamepad.left_stick_x * 0.5,
//                        -gamepad.right_stick_x * 0.4
//                )
//        );
//        drive.update();
//    }
//
//    //written as a test for kevlar tension
//    public void verticalSlideManual(Gamepad gamepad){
//        telemetry.addData("Vertical Slide: ", verticalSlide.getCurrentPosition());
//        telemetry.addData("Vertical Slide Power: ", verticalSlide.getPower());
//
//        if((gamepad.right_trigger == gamepad.left_trigger) || verticalSlide.getCurrentPosition()>=3000){
//            if(gamepad.left_trigger>0){
//                verticalSlide.setPower(-gamepad.left_trigger);
//            }
//            else {
//                verticalSlide.setPower(0.15);
//            }
//        }
//        else{
//            verticalSlide.setPower(gamepad.right_trigger-gamepad.left_trigger);
//        }
//    }
//
//
//    //----------------------------------------------------------------------------------------//
//
//    public void moveArmServo(){
//        armRight.setPosition(armPosition);
//        armLeft.setPosition(1 - armRight.getPosition());
//        telemetry.addData("arm servo: ", armRight.getPosition());
//        telemetry.addData("arm servo 1: ", armLeft.getPosition());
//    }
//
//    public void pickup(Gamepad gamepad){
//        switch (pickupStage){
//            case PICKUP_START:
//                horizontalSlide.setPower(0);
//                horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                clawServo.setPosition(clawServoOpen);
//                clawPivot.setPosition(clawPivotUp);
//                clawRotate.setPosition(clawRotateDropOff);
//                armPosition = armServoUp + 0.05;
//                moveArmServo();
//                if(gamepad.right_trigger>0) {
//                    pickupStage = PickupStage.MOVE_SLIDE;
//                }
//                break;
//
//            case MOVE_SLIDE:
//                armPosition = armServoDown-0.04;
//                moveArmServo();
//                clawPivot.setPosition(clawPivotDown);
//
//                horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                horizontalSlide.setPower(gamepad.right_trigger-gamepad.left_trigger);
//
//                //TODO UNCOMMENT********************************************************
////                if(getContourAngle_and_Position()[0]>165 && getContourAngle_and_Position()[0]<15){
////                    clawRotate.setPosition(0);
////                }
////                else{
////                    clawToPosition();
////                }
//
//                if(gamepad.dpad_left){
//                    clawRotate.setPosition(clawRotateDropOff);
//                }
//                else if(gamepad.dpad_right){
//                    clawRotate.setPosition(0);
//                }
//
//                if(gamepad.x) {
//                    pickupStage = PickupStage.ARM_DOWN;
//                    timer.reset();
//                }
//                break;
//
//
//            case ARM_DOWN:
//                armPosition = armServoDown;
//                moveArmServo();
//                horizontalSlide.setPower(gamepad.right_trigger-gamepad.left_trigger);
//                if(timer.milliseconds()>=30){
//                    pickupStage = PickupStage.CLOSE_CLAW;
//                    timer.reset();
//                }
//                break;
//
//
//            case CLOSE_CLAW:
//                clawServo.setPosition(clawServoClosed);
//                horizontalSlide.setPower(gamepad.right_trigger-gamepad.left_trigger);
//                if(timer.milliseconds()>=400){
//                    pickupStage = PickupStage.BACK_TO_START;
//                    timer.reset();
//                }
//                break;
//
//
//            case BACK_TO_START:
//                clawRotate.setPosition(clawRotateDropOff);
//                clawPivot.setPosition(clawPivotUp);
//                armPosition = armServoDown-0.11;
//                horizontalSlide.setTargetPosition(0);
//                horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                horizontalSlide.setPower(1);
//                moveArmServo();
//                if(!horizontalSlide.isBusy() && gamepad.x){
//                    pickupStage = PickupStage.ARM_UP;
//                    timer.reset();
//                }
//                else if(gamepad.y){
//                    clawServo.setPosition(clawServoOpen);
//                    pickupStage = PickupStage.MOVE_SLIDE;
//                }
//                break;
//
//            case ARM_UP:
//                armPosition = armServoUp;
//                moveArmServo();
//                if(timer.milliseconds()>=700){
//                    pickupStage = PickupStage.TRANSFER;
//                    timer.reset();
//                }
//                break;
//
//            case TRANSFER:
//                clawServo.setPosition(clawServoOpen);
//                if(timer.milliseconds()>=400){
//                    pickupStage = PickupStage.PICKUP_START;
//                    timer.reset();
//                }
//                break;
//
//
//            default:
//                pickupStage = PickupStage.PICKUP_START;
//        }
//
//    }
//
//    public void dropSample(Gamepad gamepad){
//        if(gamepad.b){
//            basketServo.setPosition(basketServoOpen);
//        }
//        else{
//            basketServo.setPosition(basketServoClosed);
//        }
//    }
//
//
//    public List<ColorBlobLocatorProcessor.Blob> contoursList() {
//        // Read the current list
//        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
//
//        //filter by aspect ratio and density
//        //ColorBlobLocatorProcessor.Util.filterByAspectRatio(2, 3.2, blobs);
//        //ColorBlobLocatorProcessor.Util.filterByDensity(0.5, 1, blobs);
//
//        //sort by size
//        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);  // filter out very small blobs.
//
//        return blobs;
//    }
//
//    public double[] getContourAngle_and_Position(){
//        List<ColorBlobLocatorProcessor.Blob> blobs = contoursList();
//
//        if (!blobs.isEmpty()) {
//            // Display the size (area) and center location for each Blob.
//            telemetry.addLine(" Area Density Aspect  Center  Angle");
//            b = blobs.get(0);
//
//            RotatedRect boxFit = b.getBoxFit();
//
//            if (boxFit.size.width < boxFit.size.height) {
//                angle = boxFit.angle + 90;
//            } else {
//                angle = boxFit.angle;
//            }
//
//            return new double[]{angle, boxFit.center.x, boxFit.center.y};
//        }
//        return new double[]{0.0,0.0,0.0};
//    }
//
//    public void clawToPosition(){
//        sampleAngle = getContourAngle_and_Position()[0] / 270;
//        clawRotate.setPosition(sampleAngle);
//        telemetry.addData("claw pivot:", clawRotate.getPosition());
//    }
//
//    }