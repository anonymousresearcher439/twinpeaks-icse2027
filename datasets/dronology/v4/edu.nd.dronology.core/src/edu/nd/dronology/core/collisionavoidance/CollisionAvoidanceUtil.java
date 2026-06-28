package edu.nd.dronology.core.collisionavoidance;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import org.apache.commons.math3.util.CombinatoricsUtils;

import edu.nd.dronology.core.collisionavoidance.guidancecommands.StopCommand;
import edu.nd.dronology.core.collisionavoidance.guidancecommands.WaypointCommand;
import edu.nd.dronology.core.goal.AbstractGoal;
import edu.nd.dronology.core.goal.IGoalSnapshot;
import edu.nd.dronology.core.goal.WaypointGoalSnapshot;

public class CollisionAvoidanceUtil {

    private static class DronePairIterator implements Iterator<DronePair> {

        Iterator<int[]> indices;
        List<DroneSnapshot> snapshots;

        DronePairIterator(List<DroneSnapshot> snapshots, Iterator<int[]> indices) {
            this.snapshots = snapshots;
            this.indices = indices;
        }

		@Override
		public boolean hasNext() {
			return indices.hasNext();
		}

		@Override
		public DronePair next() {
            int[] nextIndices = indices.next();
            int indexA = nextIndices[0];
            int indexB = nextIndices[1];
            return new DronePair(snapshots.get(indexA), snapshots.get(indexB));
		}

    }

    private static Iterator<DronePair> pairs(List<DroneSnapshot> snapshots) {
        if (snapshots.size() >= 2) {
            Iterator<int[]> indices = CombinatoricsUtils.combinationsIterator(snapshots.size(), 2);
            return new DronePairIterator(snapshots, indices);
        } else {
            return Collections.emptyIterator();
        }
    }

    public static Iterable<DronePair> findPairs(final List<DroneSnapshot> snapshots) {
        return new Iterable<DronePair>(){
        
            @Override
            public Iterator<DronePair> iterator() {
                return pairs(snapshots);
            }
            
        };
    }

    /**
     * given the drone snapshots, filter out the drones that are not flying
     * @param drones all the drone snapshots
     * @return a list with all the flying drones
     */
    public static ArrayList<DroneSnapshot> findFlyingDrones(ArrayList<DroneSnapshot> drones) {
        return drones.stream()
                .filter(drone -> "FLYING".equals(drone.getState()))
                .collect(Collectors.toCollection(ArrayList::new));
    }

    // if no active waypoint goal exists return null, otherwise return the first one it finds. Note if more than one
    // active waypoint goal exists in the set of active goals, the active goal that is returned is up to the set's
    // iterator
    public static WaypointGoalSnapshot findActiveWaypointGoal(Set<IGoalSnapshot> goals) {
        for (IGoalSnapshot goal: goals) {
            if (goal instanceof WaypointGoalSnapshot && goal.getState() == AbstractGoal.GoalState.ACTIVE) {
                return (WaypointGoalSnapshot) goal;
            }
        }
        return null;
    }

    /*
    if this method is called, we want the drone to fly to the current goal, so we will wipe out command queue and put a
    waypoint command in there unless the drone is already doing the right thing.
     */
    public static void flyToGoalIfNotAlready(DroneSnapshot drone, WaypointGoalSnapshot goal) {
        // check if we are doing the right thing: that we have one cmd in the command queue and that cmd matches the
        // current waypoint goal
        if (drone.getCommands().size() == 1) {
            if (drone.getCommands().get(0) instanceof WaypointCommand) {
                WaypointCommand wp = (WaypointCommand) drone.getCommands().get(0);
                boolean sameDest = wp.getDestination().toLlaCoordinate().equals(goal.getPosition().toLlaCoordinate());
                boolean sameSpeed = wp.getSpeed() == goal.getSpeed();
                if (sameDest && sameSpeed) {
                    return;
                }
            }
        }
        // we have now filtered out the case where we don't have to do anything. If this code runs, we need to replace
        // what's in the command queue
        drone.getCommands().clear();
        WaypointCommand cmd = new WaypointCommand(goal.getPosition().toLlaCoordinate(), goal.getSpeed());
        drone.getCommands().add(cmd);
    }

    /**
     * Make sure the drone's command queue has a stop command, overwriting the current command queue if needed.
     * @param drone to stop
     */
    public static void stopDroneIfNotStopped(DroneSnapshot drone) {
        // we don't need to do anything if the first command in the command queue is a stop command
        if (!drone.getCommands().isEmpty()) {
            if (drone.getCommands().get(0) instanceof StopCommand) {
                return;
            }
        }
        drone.getCommands().clear();
        drone.getCommands().add(new StopCommand(-1.0));
    }
}