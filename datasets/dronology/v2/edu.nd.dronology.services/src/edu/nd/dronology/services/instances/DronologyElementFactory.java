package edu.nd.dronology.services.instances;

import edu.nd.dronology.services.core.items.FlightRoute;
import edu.nd.dronology.services.core.items.IFlightRoute;
import edu.nd.dronology.services.core.items.ISimulatorScenario;
import edu.nd.dronology.services.core.items.IUAVEquipmentTypeRegistration;
import edu.nd.dronology.services.core.items.IUAVRegistration;
import edu.nd.dronology.services.core.items.IUAVTypeRegistration;
import edu.nd.dronology.services.core.items.SimulatorScenario;
import edu.nd.dronology.services.core.items.UAVEquipmentTypeRegistration;
import edu.nd.dronology.services.core.items.UAVRegistration;
import edu.nd.dronology.services.core.items.UAVTypeRegistration;

public class DronologyElementFactory {

	public static IFlightRoute createNewFlightPath() {
		return new FlightRoute();
	}

	public static IUAVRegistration createNewUAVRegistration() {
		return new UAVRegistration();
	}
	
	public static IUAVTypeRegistration createNewUAVTypeSpecification() {
		return new UAVTypeRegistration();
	}
	
	public static IUAVEquipmentTypeRegistration createNewUAVEquipmentTypeSpecification() {
		return new UAVEquipmentTypeRegistration();
	}


	public static ISimulatorScenario createNewSimulatorScenario() {
		return new SimulatorScenario();
	}

}
