package edu.nd.dronology.core.collisionavoidance.strategy;

import edu.nd.dronology.core.collisionavoidance.CollisionAvoider;
import edu.nd.dronology.core.collisionavoidance.DroneSnapshot;
import edu.nd.dronology.core.goal.WaypointGoalSnapshot;
import net.mv.logging.ILogger;
import net.mv.logging.LoggerProvider;

import java.util.ArrayList;

import static edu.nd.dronology.core.collisionavoidance.CollisionAvoidanceUtil.*;

/**
 * The StopEveryone CollisionAvoider is a failsafe that only triggers if it detects two drones are intruding into each
 * others space because something has gone wrong. Use this with mission plans that carefully navigate the drones to
 * avoid crashing into one another. StopEveryone assumes the drones will follow a mission plan that takes into account
 * where all the drones will be in space and time. When the StopEveryone CollisionAvoider is triggered the mission is
 * aborted, and humans need to land the drones manually.
 */
public class StopEveryone implements CollisionAvoider {
    private static final ILogger LOGGER = LoggerProvider.getLogger(StopEveryone.class);
    private final double threshold;

    /**
     * Initializes a newly created StopEveryone object that triggers all drones to stop whatever they're doing and hover
     * in place, if any two drones move closer than the threshold distance to one another.
     *
     * @param threshold distance in meters. If any two drones move close enough to be within the threshold distance,
     *                  then StopEveryone will command all drones to stop whatever they're doing and hover in place.
     */
    public  StopEveryone(double threshold) {
        this.threshold = threshold;
    }

    @Override
    public void avoid(ArrayList<DroneSnapshot> drones) {
        ArrayList<DroneSnapshot> flyingDrones = findFlyingDrones(drones);
        if (isSafe(flyingDrones)) {
            // fly to the goal
            for (DroneSnapshot drone : flyingDrones) {
                WaypointGoalSnapshot waypointGoal = findActiveWaypointGoal(drone.getGoals());
                if (waypointGoal != null) {
                    LOGGER.debug(drone.getName() + " had a waypoint goal: " + waypointGoal.getPosition().toLlaCoordinate());
                    flyToGoalIfNotAlready(drone, waypointGoal);
                } else {
                    LOGGER.debug(drone.getName() + " had no waypoint goal");
                }
            }
        }
        else {
            // stop everyone
            for (DroneSnapshot drone : flyingDrones) {
                LOGGER.fatal("WARNING ALL DRONES STOPPED");
                onStopTrigger(drone);
            }
        }
    }

    protected void onStopTrigger(DroneSnapshot drone) {
        stopDroneIfNotStopped(drone);
    }

    /**
     * @param flyingDrones the drones that are flying (not all drones)
     * @return true if every drone is at least threshold distance apart
     */
    protected boolean isSafe(ArrayList<DroneSnapshot> flyingDrones) {
        for (int i = 0; i < flyingDrones.size() - 1; ++i) {
            for (int j = i + 1; j < flyingDrones.size(); ++j) {
                if (isTooClose(flyingDrones.get(i), flyingDrones.get(j))) {
                    return false;
                }
            }
        }
        return true;
    }


    /**
     * Check if two drones are too close
     * @param a the first drone
     * @param b the second drone
     * @return true if the distance between the drones is less than the threshold distance
     */
    private boolean isTooClose(DroneSnapshot a, DroneSnapshot b) {
        double distance = a.getPosition().distance(b.getPosition());
        boolean result = distance < this.threshold;
        if (result) {
            LOGGER.warn("DRONES TOO CLOSE " + a.getName() + ", " + b.getName() + " distance: " + distance);
        }
        return result;
    }

}
