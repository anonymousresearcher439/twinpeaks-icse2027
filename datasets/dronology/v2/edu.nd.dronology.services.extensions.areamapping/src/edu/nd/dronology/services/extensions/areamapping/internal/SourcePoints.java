package edu.nd.dronology.services.extensions.areamapping.internal;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class SourcePoints {
	private List<Point2D.Double> sourcePoints;
	
	public SourcePoints() {
		sourcePoints = new ArrayList<>();
	}
	
	public List<Point2D.Double> getSourcePoints(){
		return Collections.unmodifiableList(sourcePoints);
	}
	
	public Point2D.Double getSourcePoint(int index){
		return sourcePoints.get(index);
	}
	
	public void setSourcePoints(List<Point2D.Double> points) {
		sourcePoints = points;
	}
	
	public void setSourcePoint(int index, Point2D.Double point) {
		sourcePoints.set(index, point);
	}
	
	public void addSourcePoint(Point2D.Double point) {
		sourcePoints.add(point);
	}
	
	public int size() {
		return sourcePoints.size();
	}
}
