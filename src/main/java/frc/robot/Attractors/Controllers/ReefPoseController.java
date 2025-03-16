package frc.robot.Attractors.Controllers;

public class ReefPoseController {
    public FaceController AlgaeHighController;    
    public FaceController AlgaeLowController;    
    public FaceController LeftCoralL3Controller;    
    public FaceController RightCoralL3Controller;    
    public FaceController RightCoralL4Controller;    
    public FaceController LeftCoralL4Controller;

    public ReefPoseController(
        FaceController algaeHigh, 
        FaceController algaeLow, 
        FaceController leftCoralL3, 
        FaceController rightCoralL3, 
        FaceController leftCoralL4,
        FaceController rightCoralL4 
    ) {
        this.AlgaeHighController = algaeHigh;
        this.AlgaeLowController = algaeLow;
        this.LeftCoralL3Controller = leftCoralL3;
        this.RightCoralL3Controller = rightCoralL3;
        this.RightCoralL4Controller = rightCoralL4;
        this.LeftCoralL4Controller = leftCoralL4;
    }
}

