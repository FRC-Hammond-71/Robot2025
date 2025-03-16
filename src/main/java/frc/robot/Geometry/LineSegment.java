package frc.robot.Geometry;

import edu.wpi.first.math.geometry.Translation2d;

public class LineSegment 
{
    public Translation2d TranslationA;
    public Translation2d TranslationB;

    public LineSegment(Translation2d a, Translation2d b)
    {
        this.TranslationA = a;
        this.TranslationB = b;
    }
    public LineSegment(LineSegment line)
    {
        this(line.TranslationA, line.TranslationB);
    }
}