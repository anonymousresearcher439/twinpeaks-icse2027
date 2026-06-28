package edu.nd.dronology.services.extensions.areamapping;

import java.util.ArrayList;
import java.util.List;

import com.google.common.base.Strings;

import edu.nd.dronology.services.core.areamapping.EdgeLla;
import edu.nd.dronology.services.extensions.areamapping.metrics.AllocationInformation;
import edu.nd.dronology.services.extensions.areamapping.metrics.Drone;
import edu.nd.dronology.services.extensions.areamapping.model.RoutePrimitive;

public class RouteCreationRunner {

	private IRouteCreator riverMapper;
	private static final int NUMBER_OF_DRONES = 5; //selector
	private static final int NUMBER_SEGMENT_FRAGMENTS = 3; //selector

	public AllocationInformation run(List<EdgeLla> side0, List<EdgeLla> side1) {
		
		

		//riverMapper = new MapRiver(side0, side1);
		//this also shouldnt be here
		riverMapper = new MapRiver();
		List<RoutePrimitive> generatedRoutes = riverMapper.generateRoutePrimitives();

		RouteSelector selector = new RouteSelector();
		//wrapper for AI input...
		selector.initialize(generatedRoutes, riverMapper.getBankList(), riverMapper.getTotalRiverSegment(), 3);
		long startTime = System.currentTimeMillis();
		AllocationInformation createdRouteAssignments = selector.generateAssignments();
		long endTime = System.currentTimeMillis();
		System.out.println("That took " + (endTime - startTime) / 1000 + " seconds");
		//show it in UI....
		
		return createdRouteAssignments;
		
	

	}
	//for testing purposes
	public static void main(String[] args) {
		RouteCreationRunner runner = new RouteCreationRunner();
		runner.run(new ArrayList<>(), new ArrayList<>());
	}


}
