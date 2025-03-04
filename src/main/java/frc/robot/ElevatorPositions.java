package frc.robot;

public enum ElevatorPositions 
{
    Stowed(0),
    L1(10),
    L2(13),
    L3(17),
    L4(22);
    // TODO: Add more!
    
    private double m_height;

    /**
     * @param height The height the elevator should extend to in inches
     */
    private ElevatorPositions(double height)
    {
        this.m_height = height;
    }

    public double getHeight()
    {
        return this.m_height;
    }
}