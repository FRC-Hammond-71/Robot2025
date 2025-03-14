package frc.robot.Subsystems.Elevator;

public class ElevatorPosition
{
    public final static ElevatorPosition Stowed = new ElevatorPosition(0);
    public final static ElevatorPosition Algae = new ElevatorPosition(1.5);
    public final static ElevatorPosition L1 = new ElevatorPosition(5);
    public final static ElevatorPosition L2 = new ElevatorPosition(3);
    public final static ElevatorPosition L3 = new ElevatorPosition(17);
    public final static ElevatorPosition L4 = new ElevatorPosition(20);
    public final static ElevatorPosition LowerAlgae = new ElevatorPosition(14);
    public final static ElevatorPosition Max = new ElevatorPosition(26);
    
    private double heightInInches;

    /**
     * @param height The height the elevator should extend to in inches!
     */
    private ElevatorPosition(double height)
    {
        this.heightInInches = height;
    }

    public double getHeight()
    {
        return this.heightInInches;
    }

    public static ElevatorPosition Arbitrary(double height)
    {
        return new ElevatorPosition(height);
    }
}