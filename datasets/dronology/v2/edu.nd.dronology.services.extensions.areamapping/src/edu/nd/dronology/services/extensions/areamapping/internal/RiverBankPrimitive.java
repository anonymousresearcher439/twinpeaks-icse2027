package edu.nd.dronology.services.extensions.areamapping.internal;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

import edu.nd.dronology.services.extensions.areamapping.model.RoutePrimitive;
import edu.nd.dronology.services.extensions.areamapping.util.Utilities;

public class RiverBankPrimitive implements SearchPatternStrategy{
	private List<SourcePoints> sourcePointsList;
	
	public RiverBankPrimitive() {
		sourcePointsList = new ArrayList<>();
	}
	
	@Override
	public void setSourcePoints(List<SourcePoints> points) {
		sourcePointsList = points;
	}
	
	private RoutePrimitive transformSourcePoints(SourcePoints sourcePoints, double APERATURE_HEIGHT, double OVERLAP_FACTOR) {
		RoutePrimitive newRoute = new RoutePrimitive();
		for(Point2D.Double entry : sourcePoints.getSourcePoints()) {
			newRoute.addRouteWaypoint(entry);
		}
		Utilities.generateImageWaypoints(newRoute, APERATURE_HEIGHT, OVERLAP_FACTOR);
		return newRoute;
	}
	
	@Override
	public List<RoutePrimitive> generateRoutePrimitive(double APERATURE_HEIGHT, double OVERLAP_FACTOR){

		List<RoutePrimitive> routes = new ArrayList<>();
		routes.add(new RoutePrimitive());
		routes.add(new RoutePrimitive());
		int counter = 0;
		for(SourcePoints source : sourcePointsList) {
			routes.set(counter, transformSourcePoints(source, APERATURE_HEIGHT, OVERLAP_FACTOR));
			counter += 1;
		}
		return routes;
	}
}
