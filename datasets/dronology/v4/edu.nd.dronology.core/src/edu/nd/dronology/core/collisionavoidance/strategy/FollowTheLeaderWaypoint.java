package edu.nd.dronology.core.collisionavoidance.strategy;

import java.util.ArrayList;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import edu.nd.dronology.core.collisionavoidance.CollisionAvoider;
import edu.nd.dronology.core.collisionavoidance.DroneSnapshot;
import edu.nd.dronology.core.collisionavoidance.guidancecommands.WaypointCommand;
import edu.nd.dronology.core.coordinate.LlaCoordinate;
import edu.nd.dronology.core.goal.WaypointGoalSnapshot;

import static edu.nd.dronology.core.collisionavoidance.CollisionAvoidanceUtil.*;

public class FollowTheLeaderWaypoint implements CollisionAvoider {

    private final Vector3D nedOffset;
    private final double defaultSpeed = 5.0;

    public FollowTheLeaderWaypoint(Vector3D nedOffset) {
        this.nedOffset = nedOffset;
    }

    public FollowTheLeaderWaypoint(double north, double east, double down) {
        this(new Vector3D(north, east, down));
    }

	@Override
	public void avoid(ArrayList<DroneSnapshot> drones) {
        DroneSnapshot leader = FollowTheLeaderNed.findLeader(drones);
        DroneSnapshot follower = FollowTheLeaderNed.findFollower(drones);

        if (leader != null) {
            double followSpeed = defaultSpeed;
            WaypointGoalSnapshot wp = findActiveWaypointGoal(leader.getGoals());
            if (wp != null) {
                flyToGoalIfNotAlready(leader, wp);
                followSpeed = wp.getSpeed();
            }

            if (follower != null) {
                LlaCoordinate followerWaypoint = leader.getPosition().findLla(this.nedOffset);
                follower.getCommands().clear();
                follower.getCommands().add(new WaypointCommand(followerWaypoint, followSpeed));
            }
        } else if (follower != null) {
            WaypointGoalSnapshot wp = findActiveWaypointGoal(follower.getGoals());
            if (wp != null) {
                flyToGoalIfNotAlready(follower, wp);
            }   
        }
	}

}