package edu.nd.dronology.services.extensions.areamapping.metrics;

public class MetricsStatistics {
	private double equalityOfTasks;
	private double allocationCoverage;
	private double downstreamRatio;
	private boolean batteryFailed;
	private int collisions;
	private double allocationScore;
	
	public MetricsStatistics(double equalityOfTasks, double allocationCoverage, double downstreamRatio, boolean batteryFailed, int collisions) {
		this.equalityOfTasks = equalityOfTasks;
		this.allocationCoverage = allocationCoverage;
		this.downstreamRatio = downstreamRatio;
		this.batteryFailed = batteryFailed;
		this.collisions = collisions;
		calculateAllocationScore();
	}
	
	private void calculateAllocationScore() {
		if(batteryFailed) {
			allocationScore = 0;
		} else {
			allocationScore = 0.25*(equalityOfTasks + allocationCoverage + downstreamRatio - collisions / 5);
		}
	}
	
	public double getEqualityOfTasks() {
		return equalityOfTasks;
	}
	
	public double getAllocationCoverage() {
		return allocationCoverage;
	}
	
	public double getDownstreamToUpstreamRatio() {
		return downstreamRatio;
	}
	
	public boolean getBatteryFailed() {
		return batteryFailed;
	}
	
	public int getCollisions() {
		return collisions;
	}
	
	public double getAllocationScore() {
		return allocationScore;
	}
}
