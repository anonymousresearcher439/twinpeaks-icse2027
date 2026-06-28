package edu.nd.dronology.services.missionplanning.plan;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import edu.nd.dronology.core.DronologyConstants;
import edu.nd.dronology.core.coordinate.LlaCoordinate;
import edu.nd.dronology.core.vehicle.IUAVProxy;
import edu.nd.dronology.services.core.items.IMissionPlan;
import edu.nd.dronology.services.core.util.DronologyServiceException;
import edu.nd.dronology.services.dronesetup.DroneSetupService;
import edu.nd.dronology.services.missionplanning.MissionExecutionException;
import edu.nd.dronology.services.missionplanning.sync.SynchronizationManager;
import edu.nd.dronology.services.missionplanning.tasks.TaskFactory;
import net.mv.logging.ILogger;
import net.mv.logging.LoggerProvider;

/**
 * 
 * Manages a mission plan. Each Mission plan has one <code>FullMissionPlan</code> instance, and one <code>UAVMissionPlan</code> instance for each UAV in the Mission plan. <br>
 * Each of the UAV's individual mission plans are composed of MissionTasks. <br>
 * Once the entire mission plan is loaded, a thread is created which checks each of the individual UAVMissionPlans to determine if they can start the next task.
 * 
 *@author Jane Cleland-Huang
 */
public class FullMissionPlan2{

	private static final ILogger LOGGER = LoggerProvider.getLogger(FullMissionPlan2.class);

	private Map<String, UAVMissionPlan> uavMissionPlans;
	private SynchronizationManager synchPointMgr;

	private IMissionPlan instructions;

	/**
	 * Constructs the CoordinatedMission instance. A mission consists of one or more UAVs, each of which has a set of assigned tasks and synchronization points.
	 * @param instructions 
	 * 
	 * @param mission
	 */
	FullMissionPlan2(IMissionPlan instructions) {
		this.instructions = instructions;
		uavMissionPlans = new HashMap<>();
		synchPointMgr = SynchronizationManager.getInstance();

	}




	/**
	 * Adds an additional UAV to the mission plan. Creates the <code>UAVMissionTasks</code> instance and passes it a reference to the <code>synchPointMgr</code>
	 * 
	 * @param uavID
	 *          the ID of the UAV
	 * @throws MissionExecutionException
	 */
	public void addUAV(String uavID) throws MissionExecutionException {
		UAVMissionPlan plan = new UAVMissionPlan(uavID, synchPointMgr);
		if (uavMissionPlans.containsKey(uavID)) {
			throw new MissionExecutionException("Mission Plan for UAV '" + uavID + "' already defined");
		}
		uavMissionPlans.put(uavID, plan);
	}

	public void removeUAV(String uavID) {
		uavMissionPlans.remove(uavID);
	}

	/**
	 * Assigns a task to a specific UAV
	 * 
	 * @param uavID
	 *          UAV Identifier
	 * @param task
	 *          Task to perform (e.g., Route, Waypoint, Synchronize, FlightPattern)
	 * @param taskID
	 *          Task specifics (e.g., specific waypoint, route name etc)
	 * @throws MissionExecutionException
	 */
	public void addTask(String uavID, String task, String taskID, Object... params) throws MissionExecutionException {
		for (UAVMissionPlan plan : uavMissionPlans.values()) {
			if (plan.getUavID().equals(uavID)) {
				plan.addTask(TaskFactory.getTask(task, uavID, taskID, params), synchPointMgr);
				return;
			}
		}
		throw new MissionExecutionException("UAVMissionPlan '" + uavID + "' not available!");
	}

	public boolean isMissionActive() {
		for (UAVMissionPlan plan : uavMissionPlans.values()) {
			if (plan.hasTasks()) {
				return true;
			}
		}
		return false;
	}

	/**
	 * Build all synch points
	 */
	private void buildAllSynchPoints() {
		uavMissionPlans.forEach((uavId, plan) -> {
			plan.buildSynchPoints();
		});
	}

	/**
	 * Activates next task in each UAV mission, if there is no unfinished active task
	 * 
	 * @throws MissionExecutionException
	 */
	public void checkAndActivateTask() throws MissionExecutionException {
		for (UAVMissionPlan plan : uavMissionPlans.values()) {
			if (!plan.hasActiveTask()) {
				plan.activateNextTask();
			}
		}
	}

	/**
	 * Expands flight pattern tasks (e.g., coordinatedTakeOff or coordinatedLanding)
	 * 
	 * @throws MissionExecutionException
	 *   
	 */
	private void expandAllTasks() throws MissionExecutionException {
		for (UAVMissionPlan plan : uavMissionPlans.values()) {
			plan.expandTaskList();
		}
	}

	public void build() throws MissionExecutionException {
		expandAllTasks();
		buildAllSynchPoints();
		synchPointMgr.activateAllSynchPoints();
		runPreChecks();

	}

	private void runPreChecks() throws MissionExecutionException {
		for (Entry<String, UAVMissionPlan> e : uavMissionPlans.entrySet()) {
			checkDistance(e.getKey(), e.getValue());
		}
	}

	private void checkDistance(String uavid, UAVMissionPlan plan) throws MissionExecutionException {
		LlaCoordinate coordinate = plan.getStartingRouteWaypoint();
		IUAVProxy uav = null;
		if (coordinate == null) {
			throw new MissionExecutionException("Error when retrieving first waypoint for uav '" + uavid + "'");
		}
		try {
			uav = DroneSetupService.getInstance().getActiveUAV(uavid);
		} catch (DronologyServiceException e) {
			throw new MissionExecutionException(e.getMessage());
		}
		double distanceToFirstWaypoint = uav.getCoordinates().distance(coordinate);
		if (distanceToFirstWaypoint > DronologyConstants.MISSION_MAX_STARTING_DISTANCE) {
			throw new MissionExecutionException(
					"Distance to first waypoint exceeds maximum safety distance: " + distanceToFirstWaypoint + "m");
		}
		LOGGER.info("Precheck passed -- Distance to first waypoint: " + distanceToFirstWaypoint);

	}

	public void cancelMission() {
		LOGGER.missionError("Mission cancelled!");
		for (UAVMissionPlan plan : uavMissionPlans.values()) {
			String uavid = plan.getUavID();
			MissionUtil.stopUAV(uavid);
		}
	}



}
