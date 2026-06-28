package edu.nd.dronology.core.collisionavoidance.strategy;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.nd.dronology.core.collisionavoidance.DroneSnapshot;
import edu.nd.dronology.core.collisionavoidance.guidancecommands.WaypointCommand;

public class StopEveryoneWaypoint extends StopEveryone {

    public static final double STOP_SPEED = 2.0;
    private Map<String, WaypointCommand> firstPos = new HashMap<>();

    public StopEveryoneWaypoint(double threshold) {
        super(threshold);
    }

    @Override
    protected void onStopTrigger(DroneSnapshot drone) {
        // if first pos contains a command for the given drone, then we are aware of a command that we want the drone to be flying 
        if (firstPos.containsKey(drone.getName())) {
            // if the drone's command queue matches the state we are looking for exactly
            if (drone.getCommands().size() == 1 && drone.getCommands().get(0) == firstPos.get(drone.getName())) {
                return;
            }
        }
        WaypointCommand stopPos = null;
        if (firstPos.containsKey(drone.getName())) {
            stopPos = firstPos.get(drone.getName());
        }
        if (stopPos == null) {
            stopPos = new WaypointCommand(drone.getPosition(), STOP_SPEED);
            firstPos.put(drone.getName(), stopPos);
        }

        drone.getCommands().clear();
        drone.getCommands().add(stopPos);
    }

    @Override
    protected boolean isSafe(ArrayList<DroneSnapshot> flyingDrones) {
        boolean isSuperSafe = super.isSafe(flyingDrones);
        if (isSuperSafe && !firstPos.isEmpty()) {
            firstPos.clear();
        }
        return isSuperSafe;
    }

}