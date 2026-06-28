package edu.nd.dronology.core.collisionavoidance.strategy;

import java.util.ArrayList;

import edu.nd.dronology.core.collisionavoidance.CollisionAvoider;
import edu.nd.dronology.core.collisionavoidance.DroneSnapshot;
import edu.nd.dronology.core.goal.WaypointGoalSnapshot;

import static edu.nd.dronology.core.collisionavoidance.CollisionAvoidanceUtil.findFlyingDrones;
import static edu.nd.dronology.core.collisionavoidance.CollisionAvoidanceUtil.findActiveWaypointGoal;
import static edu.nd.dronology.core.collisionavoidance.CollisionAvoidanceUtil.flyToGoalIfNotAlready;

public class PassThrough implements CollisionAvoider {

	@Override
	public void avoid(ArrayList<DroneSnapshot> drones) {
        ArrayList<DroneSnapshot> flyingDrones = findFlyingDrones(drones);
        for (DroneSnapshot drone : flyingDrones) {
            WaypointGoalSnapshot waypointGoal = findActiveWaypointGoal(drone.getGoals());
            if (waypointGoal != null) {
                flyToGoalIfNotAlready(drone, waypointGoal);
            }
        }
    }
    
}