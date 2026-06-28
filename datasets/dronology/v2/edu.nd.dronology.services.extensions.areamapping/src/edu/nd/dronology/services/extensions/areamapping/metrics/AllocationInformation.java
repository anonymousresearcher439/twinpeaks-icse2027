package edu.nd.dronology.services.extensions.areamapping.metrics;

import java.util.ArrayList;
import java.util.List;

public class AllocationInformation {
	private List<Drone> droneAllocations;
	private MetricsStatistics metricsStatistics;
	
	public AllocationInformation() {
		droneAllocations = new ArrayList<>();
	}
	
	public List<Drone> getDroneAllocations(){
		return droneAllocations;
	}
	
	public MetricsStatistics getMetricStatistics() {
		return metricsStatistics;
	}
	
	public void setDroneAllocations(List<Drone> droneAllocations) {
		this.droneAllocations = droneAllocations;
	}
	
	public void setMetricsStatistics(MetricsStatistics metricsStatistics) {
		this.metricsStatistics = metricsStatistics;
	}
}
