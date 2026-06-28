package edu.nd.dronology.core.collisionavoidance.strategy.onionbackend;

import edu.nd.dronology.core.collisionavoidance.DroneSnapshot;
import edu.nd.dronology.core.goal.WaypointGoalSnapshot;

import static edu.nd.dronology.core.collisionavoidance.CollisionAvoidanceUtil.*;

public class DefaultAction implements IAction {

	@Override
	public void applyAction(DroneSnapshot snapshot) {
        WaypointGoalSnapshot goal = findActiveWaypointGoal(snapshot.getGoals());
        if (goal != null) {
            flyToGoalIfNotAlready(snapshot, goal);
        }
	}

}