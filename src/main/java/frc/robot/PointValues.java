package frc.robot;

public enum PointValues {
    //--------------- CORAL POINTS --------------------
    //Do not use the AUTO points for coral. thats what the autoModifier is for.
    POINTSL1(2),
    POINTSL2(3),
    POINTSL3(4),
    POINTSL4(5),
    POINTSL1AUTO(3),
    POINTSL2AUTO(4),
    POINTSL3AUTO(6),
    POINTSL4AUTO(7),  
    // ------------- OTHER SCORING ELEMENTS ---------------
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

   PointValues(int points) {
    }
}