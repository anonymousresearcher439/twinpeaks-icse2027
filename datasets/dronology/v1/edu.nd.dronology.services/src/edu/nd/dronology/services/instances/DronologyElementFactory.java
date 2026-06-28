package edu.nd.dronology.services.instances;

import edu.nd.dronology.services.core.items.AreaMapping;
import edu.nd.dronology.services.core.items.DroneSpecification;
import edu.nd.dronology.services.core.items.FlightRoute;
import edu.nd.dronology.services.core.items.IAreaMapping;
import edu.nd.dronology.services.core.items.IDroneSpecification;
import edu.nd.dronology.services.core.items.IFlightRoute;
import edu.nd.dronology.services.core.items.IMissionPlan;
import edu.nd.dronology.services.core.items.ISimulatorScenario;
import edu.nd.dronology.services.core.items.SimulatorScenario;
import edu.nd.dronology.services.missionplanning.plan.PersistableMissionPlan;

public class DronologyElementFactory {

	public static IFlightRoute createNewFlightPath() {
		return new FlightRoute();
	}

	public static IDroneSpecification createNewDroneEqiupment() {
		return new DroneSpecification();
	}

	public static ISimulatorScenario createNewSimulatorScenario() {
		return new SimulatorScenario();
	}

	public static IAreaMapping createNewAreaMapping() {
		return new AreaMapping();
	}

	public static IMissionPlan createNewMissionPlan() {
		return new PersistableMissionPlan();
	}

}
