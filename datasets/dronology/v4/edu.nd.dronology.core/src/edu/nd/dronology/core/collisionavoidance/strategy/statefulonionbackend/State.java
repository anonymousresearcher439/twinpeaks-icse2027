package edu.nd.dronology.core.collisionavoidance.strategy.statefulonionbackend;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.apache.commons.collections4.ListUtils;

import edu.nd.dronology.core.collisionavoidance.DroneSnapshot;
import edu.nd.dronology.core.collisionavoidance.guidancecommands.Command;

/**
 * This associates a sequence of guidance commands with a UAV.
 */
public class State {

    private final ArrayList<Command> expected;

    private final String primaryKey;

    /**
     * <p>
     * A State object checks the command list of a drone snapshot and overwrites it
     * if needed.
     * </p>
     *
     * <p>
     * The actingDrone has a to-do list (accessable via
     * {@link edu.nd.dronology.core.collisionavoidance.DroneSnapshot#getCommands()
     * DroneSnapshot.getCommands()}). As the actingDrone flies, it completes
     * commands on its to-do list and removes them from the list. Instances of this
     * class look at the actingDrone's to-do list and check that it's doing the
     * "right" thing. These objects replace the acting drone's to-do list if it's
     * not.
     * </p>
     *
     * @param stateName      a name for the list of commands to carry out. For
     *                       example: "Stopping", "Slowing", "Flying"
     * @param actingDrone    the drone this state will be applied to
     * @param intruderDrones the drones that caused this state to trigger
     * @param cmdSequence    the commands that this state will ensure gets run
     */
    public State(String stateName, DroneSnapshot actingDrone, List<DroneSnapshot> intruderDrones,
            List<Command> cmdSequence) {

        this.expected = new ArrayList<>(cmdSequence);
        List<String> intruderNames = intruderDrones.stream().map(intruder -> intruder.getName())
                .collect(Collectors.toList());

        String droneName = actingDrone.getName();
        primaryKey = String.format("%s is %s to avoid: [%s]", droneName, stateName, String.join(", ", intruderNames));
    }

    /*
     * we need to ensure the UAV is doing what we expect. We look to see if the
     * current list of commands the UAV is executing is a partially completed
     * version of an expected list of commands.
     * 
     * @param uav the DroneSnapshot to check and possibly command
     */
    public void updateUav(DroneSnapshot uav) {
        ArrayList<Command> cmds = new ArrayList<>(uav.getCommands());
        /*
         * how many items would we need to add to the start of the actingDrone's list to
         * create a list that is the same size as the expected list?
         */
        int deltaSize = expected.size() - cmds.size();
        if (deltaSize < 0) {
            /*
             * if the number of items on the actingDrone's to-do list is too long then the
             * actingDrone cannot be working on the expected to-do list so we need to reset
             * the list
             */
            resetTodoList(uav.getCommands());
        } else {
            /*
             * else it's conceivable that the acting drone has finished some commands on the
             * expected to-do list. In this case, we see if we can recreate the original
             * list. Remember that the actingDrone's command list only holds the commands
             * that it has not yet finished. Since it must be the case that the finished
             * items plus the unfinished items must be everything, we can try to re-create
             * the original list by starting with the missing number of items from the
             * expected list and finishing with all the items in the actingDrone's list. If
             * the re-created to-do list matches the expected to-do list, then we assume the
             * acting drone is working on the expected list.
             */
            ArrayList<Command> virtualList = new ArrayList<>();
            int i = 0;
            while (i < deltaSize) {
                virtualList.add(expected.get(i++));
            }
            virtualList.addAll(cmds);

            if (!ListUtils.isEqualList(expected, virtualList)) {
                resetTodoList(uav.getCommands());
            }
        }
    }

    @Override
    public String toString() {
        return String.format("State(%s)", primaryKey);
    }

    @Override
    public boolean equals(Object anObject) {
        if (this == anObject) {
            return true;
        }
        if (anObject instanceof State) {
            State other = (State) anObject;
            if (this.primaryKey.equals(other.primaryKey)) {
                return ListUtils.isEqualList(other.expected, this.expected);
            }
        }
        return false;
    }

    public void resetTodoList(ArrayList<Command> todos) {
        todos.clear();
        todos.addAll(expected);
    }

}