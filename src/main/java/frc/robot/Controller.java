// package frc.robot;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import java.util.HashMap;

<<<<<<< HEAD
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.math.filter.LinearFilter;
// import edu.wpi.first.math.filter.SlewRateLimiter;

// public class Controller extends SubsystemBase {
//     enum ControlSet {
//         Testing,
//         Competition
//     }
=======
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
>>>>>>> 2d3ed2b06dcf2a3ffd5ab21b26610842e9ceb6bc

//     public final Drivetrain m_swerve = new Drivetrain();
//     public final Elevator elevator = new Elevator(40, 8, 9);
//     public final Arm m_arm = new Arm(50, 52, 51);

<<<<<<< HEAD
//     // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
//     private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(4);
//     private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

//     private final LinearFilter xFilter = LinearFilter.singlePoleIIR(0.3, 0.02);
//     private final LinearFilter yFilter = LinearFilter.singlePoleIIR(0.3, 0.02);
//     private final XboxController m_controller = new XboxController(0);
//     private double overclock = 3;

//     public String name;
//     public int port;
//     public ControlSet controlSet;
//     private HashMap<String, XboxController> registeredControllers = new HashMap<String, XboxController>();

//     public Controller(String name, int port) {
//         this.name = name;
//         this.port = port;
//     }


//     public void registerDevice(String name) {
//         if (name == "Operator") {
//             if (!this.registeredControllers.containsKey(name)) {
//                 registeredControllers.put(this.name, new XboxController(this.port));
//             }
//         } else if (name == "Driver") {
//             if (!this.registeredControllers.containsKey(name)) {
//                 registeredControllers.put(this.name, new XboxController(this.port));
//             }
//         } else {
//             System.err.println("Invalid Controller Name");
//         }
//     }

//     private void TestingDrive(boolean fieldRelative) {
//         if (m_controller.getLeftBumperButton()) {
//             overclock = 3;
//         }

//         if (m_controller.getBButton()) {
//             this.m_arm.scoreAlgae();
//         } else if (m_controller.getXButton()) {
//             this.m_arm.intakeAlgae();
//         } else {
//             m_arm.stopAlgae();
//         }

//         if (m_controller.getYButton()) {
//             this.m_arm.intakeCoral();
//         } else {
//             this.m_arm.stopCoral();
//         }

//         if (m_controller.getAButton()) {
//             this.m_arm.scoreCoral();
//         } else {
//             this.m_arm.stopCoral();
//         }

//         if (m_controller.getPOV() == 90) {
//             this.elevator.setPositions(ElevatorPosition.Stowed);
//         }
//         if (m_controller.getPOV() == 180) {
//             this.elevator.setPositions(ElevatorPosition.L2);
//         }
//         if (m_controller.getPOV() == 270) {
//             this.elevator.setPositions(ElevatorPosition.L3);
//         }
//         if (m_controller.getPOV() == 0) {
//             this.elevator.setPositions(ElevatorPosition.L4);
//         }

//     }

//     public void setControlSet(ControlSet controlSet) {
//         this.controlSet = controlSet;
//     }

//     public void Drive() {
//         switch (controlSet) {

//             case Testing:
//                 // Get the x speed. We are inverting this because Xbox controllers return
//                 // negative values when we push forward.
//                 final var xSpeed = xFilter
//                         .calculate(Math.pow(MathUtil.applyDeadband(-m_controller.getLeftY(), 0.10), 3)) * (overclock);
//                 // Get the y speed or sideways/strafe speed. We are inverting this because
//                 // we want a positive value when we pull to the left. Xbox controllers
//                 // return positive values when you pull to the right by default.
//                 final var ySpeed = yFilter
//                         .calculate(Math.pow(MathUtil.applyDeadband(-m_controller.getLeftX(), 0.10), 3)) * overclock;

//                 // Get the rate of angular rotation. We are inverting this because we want a
//                 // positive value when we pull to the left (remember, CCW is positive in
//                 // mathematics). Xbox controllers return positive values when you pull to
//                 // the right by default.
//                 // final var rot =
//                 // -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(),
//                 // 0.1)) * Drivetrain.kMaxAngularSpeed;

//                 double rot = Math.pow(MathUtil.applyDeadband(-m_controller.getRightX(), 0.10), 3);

//                 ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
//                 final boolean fieldRelative = true;


//                 TestingDrive(fieldRelative);
//                 m_swerve.Drive(speeds, fieldRelative);

//                 break;
//             case Competition:

//                 break;
//         }
//     }

// }
=======
    enum ControlSet {
        Testing,
        Competition
    }

    private final Drivetrain m_swerve = new Drivetrain();
    private final Elevator elevator = new Elevator(40, 8, 9);
    private final Arm m_arm = new Arm(50, 52, 51);

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(4);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(4);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private final LinearFilter xFilter = LinearFilter.singlePoleIIR(0.3, 0.02);
    private final LinearFilter yFilter = LinearFilter.singlePoleIIR(0.3, 0.02);
    private final XboxController m_controller = new XboxController(0);
    private double overclock = 3;

    public String name;
    public int port;
    public ControlSet controlSet;
    private HashMap<String, XboxController> registeredControllers = new HashMap<String, XboxController>();

    public Controller(String name, int port) {
        this.name = name;
        this.port = port;
    }


    public void registerDevice(String name) {
        if (name == "Operator") {
            if (!this.registeredControllers.containsKey(name)) {
                registeredControllers.put(this.name, new XboxController(this.port));
            }
        } else if (name == "Driver") {
            if (!this.registeredControllers.containsKey(name)) {
                registeredControllers.put(this.name, new XboxController(this.port));
            }
        } else {
            System.err.println("Invalid Controller Name");
        }
    }

    private void TestingDrive(boolean fieldRelative) {
        if (m_controller.getLeftBumperButton()) {
            overclock = 3;
        }

        if (m_controller.getBButton()) {
            this.m_arm.scoreAlgae();
        } else if (m_controller.getXButton()) {
            this.m_arm.intakeAlgae();
        } else {
            m_arm.stopAlgae();
        }

        if (m_controller.getYButton()) {
            this.m_arm.intakeCoral();
        } else {
            this.m_arm.stopCoral();
        }

        if (m_controller.getAButton()) {
            this.m_arm.scoreCoral();
        } else {
            this.m_arm.stopCoral();
        }

        if (m_controller.getPOV() == 90) {
            this.elevator.setPositions(ElevatorPositions.Stowed);
        }
        if (m_controller.getPOV() == 180) {
            this.elevator.setPositions(ElevatorPositions.L2);
        }
        if (m_controller.getPOV() == 270) {
            this.elevator.setPositions(ElevatorPositions.L3);
        }
        if (m_controller.getPOV() == 0) {
            this.elevator.setPositions(ElevatorPositions.L4);
        }

    }

    public void setControlSet(ControlSet controlSet) {
        this.controlSet = controlSet;
    }

    public void Drive() {
        switch (controlSet) {

            case Testing:
                // Get the x speed. We are inverting this because Xbox controllers return
                // negative values when we push forward.
                final var xSpeed = xFilter
                        .calculate(Math.pow(MathUtil.applyDeadband(-m_controller.getLeftY(), 0.10), 3)) * (overclock);
                // Get the y speed or sideways/strafe speed. We are inverting this because
                // we want a positive value when we pull to the left. Xbox controllers
                // return positive values when you pull to the right by default.
                final var ySpeed = yFilter
                        .calculate(Math.pow(MathUtil.applyDeadband(-m_controller.getLeftX(), 0.10), 3)) * overclock;

                // Get the rate of angular rotation. We are inverting this because we want a
                // positive value when we pull to the left (remember, CCW is positive in
                // mathematics). Xbox controllers return positive values when you pull to
                // the right by default.
                // final var rot =
                // -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(),
                // 0.1)) * Drivetrain.kMaxAngularSpeed;

                double rot = Math.pow(MathUtil.applyDeadband(-m_controller.getRightX(), 0.10), 3);

                ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
                final boolean fieldRelative = true;


                TestingDrive(fieldRelative);
                m_swerve.Drive(speeds, fieldRelative);

                break;
            case Competition:

                break;
        }
    }

}
>>>>>>> 2d3ed2b06dcf2a3ffd5ab21b26610842e9ceb6bc
