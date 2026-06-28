package edu.nd.dronology.core.collisionavoidance.strategy;

import static edu.nd.dronology.core.collisionavoidance.CollisionAvoidanceUtil.findActiveWaypointGoal;
import static edu.nd.dronology.core.collisionavoidance.CollisionAvoidanceUtil.findFlyingDrones;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.stream.Collectors;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import edu.nd.dronology.core.collisionavoidance.CollisionAvoider;
import edu.nd.dronology.core.collisionavoidance.DroneSnapshot;
import edu.nd.dronology.core.collisionavoidance.guidancecommands.Command;
import edu.nd.dronology.core.collisionavoidance.guidancecommands.NedCommand;
import edu.nd.dronology.core.collisionavoidance.guidancecommands.StopCommand;
import edu.nd.dronology.core.collisionavoidance.guidancecommands.WaypointCommand;
import edu.nd.dronology.core.collisionavoidance.strategy.statefulonionbackend.State;
import edu.nd.dronology.core.coordinate.LlaCoordinate;
import edu.nd.dronology.core.goal.WaypointGoalSnapshot;

public class StatefulOnion implements CollisionAvoider {

    List<Layer> layers = Arrays.asList(new StopLayer(), new NormalLayer());

    Map<String, StateAndPriority> droneStates = new HashMap<>();

    @Override
    public void avoid(ArrayList<DroneSnapshot> drones) {
        ArrayList<DroneSnapshot> flyingDrones = findFlyingDrones(drones);
        for (int i = 0; i < flyingDrones.size(); ++i) {
            DroneSnapshot selected = flyingDrones.get(i);
            ArrayList<DroneSnapshot> others = new ArrayList<>(flyingDrones);
            others.remove(i);

            StateAndPriority active = findPriorityLayer(selected, others);

            /*
             * if we previously had a state, see if it takes priority
             */
            if (droneStates.containsKey(selected.getName()) && Objects.nonNull(droneStates.get(selected.getName()))) {
                StateAndPriority tmp = droneStates.get(selected.getName());
                if (!active.equals(tmp) && tmp.priority < active.priority) {
                    // in this case we are sticking to the old state we found previously
                    active = tmp;
                } else if (!active.equals(tmp)) {
                    // in this case we are changing states so we need to forece a command list reset
                    active.state.resetTodoList(selected.getCommands());
                }
            } else {
                // if this is the first time we are activating this state, reset the command list
                active.state.resetTodoList(selected.getCommands());
            }
            droneStates.put(selected.getName(), active);
            active.state.updateUav(selected);
        }
    }

    private StateAndPriority findPriorityLayer(DroneSnapshot own, List<DroneSnapshot> others) {
        int i = 0;
        while (i < layers.size()) {
            if (layers.get(i).isActive(own, others)) {
                return new StateAndPriority(layers.get(i).buildState(own, others), i);
            }
            i += 1;
        }

        int last = layers.size() - 1;
        return new StateAndPriority(layers.get(last).buildState(own, others), last);
    }

    class StateAndPriority {
        public final State state;
        public final int priority;

        StateAndPriority(State state, int priority) {
            this.state = state;
            this.priority = priority;
        }

        @Override
        public boolean equals(Object anObject) {
            if (this == anObject) {
                return true;
            }
            if (anObject instanceof StateAndPriority) {
                StateAndPriority other = (StateAndPriority) anObject;
                return this.state.equals(other.state) && this.priority == other.priority;
            }
            return false;
        }
    }

    interface Layer {
        boolean isActive(DroneSnapshot own, List<DroneSnapshot> others);

        State buildState(DroneSnapshot own, List<DroneSnapshot> others);
    }

    class NormalLayer implements Layer {

        @Override
        public boolean isActive(final DroneSnapshot own, List<DroneSnapshot> others) {
            return true;

        }

        @Override
        public State buildState(DroneSnapshot own, List<DroneSnapshot> others) {
            WaypointGoalSnapshot waypointGoal = findActiveWaypointGoal(own.getGoals());
            List<Command> cmds;
            if (waypointGoal != null) {
                LlaCoordinate destination = waypointGoal.getPosition().toLlaCoordinate();
                cmds = Arrays.asList(new WaypointCommand(destination, waypointGoal.getSpeed()), new StopCommand(0));
            } else {
                cmds = Arrays.asList(new StopCommand(0));
            }
            return new State("FLYING", own, Collections.<DroneSnapshot>emptyList(), cmds);
        }

    }

    class StopLayer implements Layer {
        private final double ACTIVATION_THRESHOLD = 40.0;
        private final double REVERSE_SPEED = 2.0;

        @Override
        public boolean isActive(DroneSnapshot own, List<DroneSnapshot> others) {
            Optional<DroneSnapshot> intruder = others.stream().filter((DroneSnapshot drone) -> {
                return drone.getPosition().distance(own.getPosition()) <= ACTIVATION_THRESHOLD;
            }).findAny();

            return intruder.isPresent();
        }

        @Override
        public State buildState(DroneSnapshot own, List<DroneSnapshot> others) {
            ArrayList<DroneSnapshot> intruders = others.stream().filter((DroneSnapshot other) -> {
                double distance = other.getPosition().distance(own.getPosition());
                return distance <= ACTIVATION_THRESHOLD;
            }).collect(Collectors.toCollection(ArrayList::new));

            // sort intruders by their distance from own. Nearest first.
            Collections.sort(intruders, (DroneSnapshot d1, DroneSnapshot d2) -> {
                double d1Distance = d1.getPosition().distance(own.getPosition());
                double d2Distance = d2.getPosition().distance(own.getPosition());
                return Double.compare(d1Distance, d2Distance);
            });

            List<Command> cmds = Arrays.asList(new NedCommand(findProjectedRetrograde(own), 1.0), new StopCommand(0));
            return new State("STOPPING", own, intruders, cmds);
        }

        private Vector3D findProjectedRetrograde(DroneSnapshot own) {
            Vector3D retrograde = own.getVelocity().negate();
            return findProjected(retrograde);
        }

        private Vector3D findProjected(Vector3D vec) {
            Vector3D projected = new Vector3D(vec.getX(), vec.getY(), 0.0);
            if (Vector3D.ZERO.equals(projected)) {
                return Vector3D.ZERO;
            }
            double projectionSpeed = Math.min(REVERSE_SPEED, projected.distance(Vector3D.ZERO));
            return projected.normalize().scalarMultiply(projectionSpeed);
        }

    }

}