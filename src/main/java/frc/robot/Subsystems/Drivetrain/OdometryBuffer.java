package frc.robot.Subsystems.Drivetrain;

/**
 * A "high-performance" circular queue for swerve module odometry.
 * Designed for measuring swerve-drive module positions at 250Hz or more.
 */
public class OdometryBuffer {

    public static final int kOutputSize = 10;
    public static final int kOutputTimestamp = 9;
    public static final int kOutputGyroHeading = 8;
    public static final int kOutputFrontLeftDrivePosition = 0;
    public static final int kOutputFrontLeftAzimuthRotation = 1;
    public static final int kOutputFrontRightDrivePosition = 2;
    public static final int kOutputFrontRightAzimuthRotation = 3;
    public static final int kOutputBackLeftDrivePosition = 4;
    public static final int kOutputBackLeftAzimuthRotation = 5;
    public static final int kOutputBackRightDrivePosition = 6;
    public static final int kOutputBackRightAzimuthRotation = 7;

    // Arrays for storing the drive positions and azimuth rotations of each module
    private final double[] frontLeftDrivePositionBuffer;
    private final double[] frontLeftAzimuthRotationInDegBuffer;

    private final double[] frontRightDrivePositionBuffer;
    private final double[] frontRightAzimuthRotationInDegBuffer;

    private final double[] backLeftDrivePositionBuffer;
    private final double[] backLeftAzimuthRotationInDegBuffer;

    private final double[] backRightDrivePositionBuffer;
    private final double[] backRightAzimuthRotationInDegBuffer;

    private final double[] gyroHeadingInDegBuffer;

    private final double[] timestampBuffer;
    private final int capacity; 

    private volatile int head = 0;
    private volatile int tail = 0;

    public OdometryBuffer(int capacity) 
    {
        this.capacity = capacity;

        this.frontLeftDrivePositionBuffer = new double[capacity];
        this.frontLeftAzimuthRotationInDegBuffer = new double[capacity];

        this.frontRightDrivePositionBuffer = new double[capacity];
        this.frontRightAzimuthRotationInDegBuffer = new double[capacity];

        this.backLeftDrivePositionBuffer = new double[capacity];
        this.backLeftAzimuthRotationInDegBuffer = new double[capacity];

        this.backRightDrivePositionBuffer = new double[capacity];
        this.backRightAzimuthRotationInDegBuffer = new double[capacity];

        this.gyroHeadingInDegBuffer = new double[capacity];
        
        this.timestampBuffer = new double[capacity];
    }

    public int length()
    {
        return Math.abs(Math.abs(this.capacity - this.head) - Math.abs(this.capacity - this.tail));
    }

    public synchronized void add(
        double frontLeftDrivePosition, 
        double frontLeftAzimuthRotationInDeg,
        double frontRightDrivePosition, 
        double frontRightAzimuthRotationInDeg,
        double backLeftDrivePosition, 
        double backLeftAzimuthRotationInDeg,
        double backRightDrivePosition, 
        double backRightAzimuthRotationInDeg,
        double gyroHeadingInDeg,
        double timestamp) 
    {
        int localHead = head;

        // Store the data for each swerve module
        frontLeftDrivePositionBuffer[localHead] = frontLeftDrivePosition;
        frontLeftAzimuthRotationInDegBuffer[localHead] = frontLeftAzimuthRotationInDeg;

        frontRightDrivePositionBuffer[localHead] = frontRightDrivePosition;
        frontRightAzimuthRotationInDegBuffer[localHead] = frontRightAzimuthRotationInDeg;

        backLeftDrivePositionBuffer[localHead] = backLeftDrivePosition;
        backLeftAzimuthRotationInDegBuffer[localHead] = backLeftAzimuthRotationInDeg;

        backRightDrivePositionBuffer[localHead] = backRightDrivePosition;
        backRightAzimuthRotationInDegBuffer[localHead] = backRightAzimuthRotationInDeg;

        gyroHeadingInDegBuffer[localHead] = gyroHeadingInDeg;
        
        // Store the timestamp
        timestampBuffer[localHead] = timestamp;

        head = (localHead + 1) % capacity; 
        if (head == tail) tail = (tail + 1) % capacity;
    }

    /**
     * Retrieves the oldest entries from the buffer.
     * @param outputBuffer A pre-allocated double array where:
     *                     outputBuffer[0] = frontLeftDrivePosition,
     *                     outputBuffer[1] = frontLeftAzimuthRotationInDeg,
     *                     outputBuffer[2] = frontRightDrivePosition,
     *                     outputBuffer[3] = frontRightAzimuthRotationInDeg,
     *                     outputBuffer[4] = backLeftDrivePosition,
     *                     outputBuffer[5] = backLeftAzimuthRotationInDeg,
     *                     outputBuffer[6] = backRightDrivePosition,
     *                     outputBuffer[7] = backRightAzimuthRotationInDeg,
     *                     outputBuffer[8] = Gyro heading in deg.
     *                     outputBuffer[9] = Timestamp.
     * @return True if data was retrieved, false if the buffer is empty.
     */
    public synchronized boolean poll(double[] outputBuffer) {
        int localTail = tail;
    
        if (localTail == head) return false;
    
        outputBuffer[kOutputFrontLeftDrivePosition] = frontLeftDrivePositionBuffer[localTail];
        outputBuffer[kOutputFrontLeftAzimuthRotation] = frontLeftAzimuthRotationInDegBuffer[localTail];
        outputBuffer[kOutputFrontRightDrivePosition] = frontRightDrivePositionBuffer[localTail];
        outputBuffer[kOutputFrontRightAzimuthRotation] = frontRightAzimuthRotationInDegBuffer[localTail];
        outputBuffer[kOutputBackLeftDrivePosition] = backLeftDrivePositionBuffer[localTail];
        outputBuffer[kOutputBackLeftAzimuthRotation] = backLeftAzimuthRotationInDegBuffer[localTail];
        outputBuffer[kOutputBackRightDrivePosition] = backRightDrivePositionBuffer[localTail];
        outputBuffer[kOutputBackRightAzimuthRotation] = backRightAzimuthRotationInDegBuffer[localTail];
        
        outputBuffer[kOutputGyroHeading] = gyroHeadingInDegBuffer[localTail];
        outputBuffer[kOutputTimestamp] = timestampBuffer[localTail];
    
        tail = (localTail + 1) % capacity;
    
        return true;
    }    

    public synchronized boolean isEmpty() {
        return head == tail;
    }
}
