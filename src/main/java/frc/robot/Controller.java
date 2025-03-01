package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;

import org.w3c.dom.NameList;

import edu.wpi.first.wpilibj.XboxController;

public class Controller extends SubsystemBase {

    enum ControlSet {
        Testing,
        Competition
    }
    
    public String name;
    public int port;
    public ControlSet controlSet;
    private HashMap<String, XboxController> registeredControllers = new HashMap<String,XboxController>();

    private Controller(String name, int port) {
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
    
    public void setControlSet(ControlSet controlSet) {
        this.controlSet = controlSet;
    }

    public void template() {
        switch (controlSet) {

            case Testing:

            break;
            case Competition:
            
            break;
        }
    }
    

    



    
}
