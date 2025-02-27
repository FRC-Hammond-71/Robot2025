package frc.robot;

public enum PointValues {
    //first set is during Teleop, Second set is during Auto,
    POINTSPROCESSOR(6),
    POINTSNET(4),
    POINTSLEAVE(2),
    //First is shallow, second is deep.
    POINTSCLIMBING(6),
    POINTSPARKING(2),

    // ------------ RANKING/COOP POINTS -----------------

    POINTSCOOP(1),
    POINTSCORALRP(1),
    POINTSBARGERP(1),

    POINTSWIN(3),
    POINTSTIE(1);



    // TODO: Add more!
    
    private double p_points;

    PointValues(int points) {
        @param points; //points. dum dum 
        this.p_points = points;

    }
}