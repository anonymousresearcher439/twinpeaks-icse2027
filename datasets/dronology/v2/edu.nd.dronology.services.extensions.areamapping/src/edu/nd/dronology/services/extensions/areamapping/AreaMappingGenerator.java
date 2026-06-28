package edu.nd.dronology.services.extensions.areamapping;

import java.util.List;

import edu.nd.dronology.services.core.areamapping.EdgeLla;
import edu.nd.dronology.services.core.areamapping.GeneratedMappedArea;
import edu.nd.dronology.services.core.items.IAreaMapping;
import edu.nd.dronology.services.extensions.areamapping.metrics.AllocationInformation;
import edu.nd.dronology.services.extensions.areamapping.metrics.Drone;

public class AreaMappingGenerator {

	private IAreaMapping mapping;

	public AreaMappingGenerator(IAreaMapping mapping) {
		this.mapping = mapping;

	}

	public GeneratedMappedArea generateMapping() {
		// TODO: implement me....

		List<EdgeLla> side0 = mapping.getMappedPoints(0);
		List<EdgeLla> side1 = mapping.getMappedPoints(1);

		RouteCreationRunner routeCreationRunner = new RouteCreationRunner();
		AllocationInformation results = routeCreationRunner.run(side0, side1);

		List<Drone> allocations = results.getDroneAllocations();

		// convert to list of waypoints/lla coordinates!
		return new GeneratedMappedArea();

	}
}
