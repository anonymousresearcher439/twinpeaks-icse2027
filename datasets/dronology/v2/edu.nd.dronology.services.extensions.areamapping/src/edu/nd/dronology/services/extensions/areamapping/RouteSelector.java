package edu.nd.dronology.services.extensions.areamapping;

import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Path2D.Double;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.omg.CORBA.PRIVATE_MEMBER;

import edu.nd.dronology.services.extensions.areamapping.internal.RiverBank;
import edu.nd.dronology.services.extensions.areamapping.metrics.AllocationInformation;
import edu.nd.dronology.services.extensions.areamapping.metrics.Drone;
import edu.nd.dronology.services.extensions.areamapping.metrics.MetricsRunner;
import edu.nd.dronology.services.extensions.areamapping.metrics.MetricsStatistics;
import edu.nd.dronology.services.extensions.areamapping.metrics.MetricsUtilities;
import edu.nd.dronology.services.extensions.areamapping.model.RoutePrimitive;
import edu.nd.dronology.services.extensions.areamapping.util.Utilities;

public class RouteSelector {

	private static final double APERATURE_WIDTH = 10;
	private static final double APERATURE_HEIGHT= 0.8* APERATURE_WIDTH;
	private static final double OVERLAP_FACTOR = 0.7;
	private int availableDrones;
	private List<RoutePrimitive> routePrimitives;
	private MetricsRunner metricsRunner;

	public void initialize(List<RoutePrimitive>routePrimitives, List<RiverBank> bankList, Path2D.Double totalRiverSegment, int availableDrones) {
		// TODO Auto-generated method stub
		routePrimitives = Utilities.splitRoutePrimitives(routePrimitives, 4, APERATURE_HEIGHT, OVERLAP_FACTOR);
		this.routePrimitives = routePrimitives;
		this.availableDrones = availableDrones;
		metricsRunner = new MetricsRunner(routePrimitives, totalRiverSegment, bankList, APERATURE_WIDTH, APERATURE_HEIGHT);
	}
	
	//GeneratedRouteAssignment....
	private List<Drone> generateRandomAssingments(){
		Set<Integer> assignedRoutes = new HashSet<>();
		availableDrones = 3;
		int droneNum;
		int routeNum;
		//should it be size() or size()-1
		int routeAssignmentNum = MetricsUtilities.generateRandomNumber(routePrimitives.size()-1, 1);
		List<Drone> droneList = new ArrayList<>();
		for(int i = 0; i < availableDrones; i++) {
			droneList.add(new Drone());
			droneList.get(i).setDroneHomeLocation(new Point2D.Double(4639658.290263815, -7163020.664734639));
			droneList.get(i).setDroneStartPoint(new Point2D.Double(4639658.290263815, -7163020.664734639));;
		}
		while(assignedRoutes.size() < routeAssignmentNum) {
			//assign drone routes in here
			droneNum = MetricsUtilities.generateRandomNumber(2,0);
			routeNum = MetricsUtilities.generateRandomNumber(routeAssignmentNum, 0);
			while(assignedRoutes.contains(routeNum)) {
				routeNum = MetricsUtilities.generateRandomNumber(routeAssignmentNum, 0);
			}
			droneList.get(droneNum).getDroneRouteAssignment().add(routePrimitives.get(routeNum));
			assignedRoutes.add(routeNum);
		}
		return droneList;
	}
	
	private MetricsStatistics generateMetricsStatistics(List<Drone> drones) {
		metricsRunner.setDroneAssignments(drones);
		return metricsRunner.runMetrics();
	}
	
	//use a loop to pick best route
	//return wrapper for list<drone> and metrics
	public AllocationInformation generateAssignments() {
		// loop to create assignments and check for best assignment
		AllocationInformation finalAllocation = new AllocationInformation();
		AllocationInformation currentAllocation = new AllocationInformation();
		List<Drone> assignments = generateRandomAssingments();
		finalAllocation.setDroneAllocations(assignments);
		finalAllocation.setMetricsStatistics(generateMetricsStatistics(assignments));
		for(int i = 1; i < 100; i++) {
			assignments = generateRandomAssingments();
			currentAllocation.setDroneAllocations(assignments);
			currentAllocation.setMetricsStatistics(generateMetricsStatistics(assignments));
			if(finalAllocation.getMetricStatistics().getAllocationScore() < currentAllocation.getMetricStatistics().getAllocationScore()) {
				finalAllocation = currentAllocation;
			}
		}
		return finalAllocation;
	}
	

}
