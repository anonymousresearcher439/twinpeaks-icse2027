package edu.nd.dronology.services.extensions.areamapping.metrics;

import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Vector;

import edu.nd.dronology.services.extensions.areamapping.internal.Geometry;
import edu.nd.dronology.services.extensions.areamapping.internal.ImageWaypoint;
import edu.nd.dronology.services.extensions.areamapping.internal.ImageWaypoints;
import edu.nd.dronology.services.extensions.areamapping.internal.RiverBank;
import edu.nd.dronology.services.extensions.areamapping.model.RoutePrimitive;

public class MetricsUtilities {
	 /**
     * This function generates circular ellipses for each image waypoint. 
     * @param route
     * @param APERATURE_WIDTH
     * @return vector of circular Ellipse2D.Double 
     */	
	public static Vector<Ellipse2D.Double> generateIWPCircles(RoutePrimitive route, double APERATURE_WIDTH){
    	Vector<Ellipse2D.Double> circles = new Vector<>();
    	ImageWaypoints imageWaypoints = route.getIWP();
    	for(ImageWaypoint entry : imageWaypoints.get()) {
    		Ellipse2D.Double newCircle = new Ellipse2D.Double(entry.getWaypoint().getX()-APERATURE_WIDTH / 2, entry.getWaypoint().getY() - APERATURE_WIDTH / 2, APERATURE_WIDTH, APERATURE_WIDTH);
    		circles.add(newCircle);
    	}
    	return circles;
    }
    
    public static Vector<Path2D.Double> generateIWPRectangles(RoutePrimitive route, double APERATURE_WIDTH, double APERATURE_HEIGHT){
    	Vector<Path2D.Double> coverageRectangles = new Vector<>();
    	for(ImageWaypoint entry : route.getIWP().get()) {
    		coverageRectangles.add(generateIWPRectangle(entry, APERATURE_WIDTH, APERATURE_HEIGHT));
    	}
    	return coverageRectangles;
    }
    
    private static Path2D.Double generateIWPRectangle(ImageWaypoint imagePoint, double APERATURE_WIDTH, double APERATURE_HEIGHT){
    	Path2D.Double coverageRectangle = new Path2D.Double();
    	Point2D.Double midpoint1 = new Point2D.Double();
    	Point2D.Double midpoint2 = new Point2D.Double();
    	Point2D.Double corner1 = new Point2D.Double();
    	Point2D.Double corner2 = new Point2D.Double();
    	Point2D.Double corner3 = new Point2D.Double();
    	Point2D.Double corner4 = new Point2D.Double();
    	double dxHeight = APERATURE_HEIGHT / 2 * Math.cos(imagePoint.getOrientationAngle());
    	double dyHeight = APERATURE_HEIGHT / 2 * Math.sin(imagePoint.getOrientationAngle());
    	double dxWidth = APERATURE_WIDTH / 2 * Math.cos(imagePoint.getOrientationAngle() + Math.PI / 2);
    	double dyWidth = APERATURE_WIDTH / 2 * Math.sin(imagePoint.getOrientationAngle() + Math.PI / 2);
    	midpoint1.setLocation(imagePoint.getWaypoint().getX() + dxHeight, imagePoint.getWaypoint().getY() + dyHeight);
    	midpoint2.setLocation(imagePoint.getWaypoint().getX() - dxHeight, imagePoint.getWaypoint().getY() - dyHeight);
    	corner1.setLocation(midpoint1.getX() + dxWidth, midpoint1.getY() + dyWidth);
    	corner2.setLocation(midpoint1.getX() - dxWidth, midpoint1.getY() - dyWidth);
    	corner3.setLocation(midpoint2.getX() - dxWidth, midpoint2.getY() - dyWidth);
    	corner4.setLocation(midpoint2.getX() + dxWidth, midpoint2.getY() + dyWidth);
    	coverageRectangle.moveTo(corner1.getX(), corner1.getY());
    	coverageRectangle.lineTo(corner2.getX(), corner2.getY());
    	coverageRectangle.lineTo(corner3.getX(), corner3.getY());
    	coverageRectangle.lineTo(corner4.getX(), corner4.getY());
    	coverageRectangle.closePath();
    	return coverageRectangle;
    }
    
	public static int generateRandomNumber(int max, int min) {
		Random random = new Random();
		return random.nextInt((max - min) + 1) + min;
	}
    
    /**
     * This function generates a pseudorandom point.
     * @param minPoint
     * @param maxPoint
     * @return Point2D.Double pseudorandom point
     */
    public static Point2D.Double generateRandomPoint(Point2D.Double minPoint, Point2D.Double maxPoint){
    	Point2D.Double newPoint = new Point2D.Double();
    	double minLat = minPoint.getX();
    	double maxLat = maxPoint.getX();
    	double minLong = minPoint.getY();
    	double maxLong = maxPoint.getY();
    	Random random = new Random();
    	double newX = (random.nextDouble()*(maxLat + 1 - minLat)) + minLat;
    	double newY = (random.nextDouble()*(maxLong + 1 - minLong)) + minLong;
    	newPoint.setLocation(newX, newY);
    	return newPoint;
    }
    
    //assumes drone startPoint and home location are stored in cartesian coordinates
    public static double totalDroneDistance(Drone drone) {
    	double totalDistance = 0;
    	DroneRouteAssignment routeAssignment = drone.getDroneRouteAssignment();
    	Point2D.Double startPoint = drone.getDroneStartPoint();
    	for(RoutePrimitive route : routeAssignment.get()) {
    		totalDistance += Geometry.findCartesianDistance(startPoint, route.getRouteStartPoint());
    		totalDistance += route.getRouteDistance();
    		startPoint = route.getRouteEndPoint();
    	}
    	
    	return totalDistance;
    }
    
    public static double equalityOfTasks(List<Drone> drones) {
    	double maxDistance = -Double.MAX_VALUE;
    	double minDistance = Double.MAX_VALUE;
    	double distance;
    	for(Drone drone : drones) {
    		distance = totalDroneDistance(drone);
    		if(distance < minDistance) {
    			minDistance = distance;
    		}
    		if(maxDistance < distance) {
    			maxDistance = distance;
    		}
    	}	
    	return minDistance / maxDistance;
    }
    
    /**
     * This function calculates coverage statistics for the chosen RoutePrimitive objects.
     * @param routes
     * @param coveragePoints
     * @param APERATURE_WIDTH
     * @return CoverageStatistics
     */
    public static double calculateRouteCoverage(List<Drone> drones, Path2D.Double totalRiverSegment, List<RiverBank> bankList, double APERATURE_WIDTH, double APERATURE_HEIGHT) {
    	Point2D.Double minBound = new Point2D.Double();
		Point2D.Double maxBound = new Point2D.Double();
		Vector<Point2D.Double> bounds = Geometry.simpleRiverBoundingRectangle(bankList);
		Vector<Point2D.Double> coveragePoints = new Vector<>();
		minBound = bounds.get(0);
		maxBound = bounds.get(1);
		for(int i = 0; i < 2000; i++) {
			Point2D.Double newPoint = MetricsUtilities.generateRandomPoint(minBound, maxBound);
			while(!totalRiverSegment.contains(newPoint)) {
				newPoint = MetricsUtilities.generateRandomPoint(minBound, maxBound);
			}
			coveragePoints.add(newPoint);
		}
    	double coverageFraction;
    	double missedPoints = 0;
    	boolean covered = false;
    	List<RoutePrimitive> routes = new ArrayList<>();
    	for(Drone drone : drones) {
    		routes.addAll(drone.getDroneRouteAssignment().get());
    	}
    	Vector<Path2D.Double> imageWaypointRectangles = new Vector<>();
    	for(RoutePrimitive route : routes) {
    		imageWaypointRectangles.addAll(generateIWPRectangles(route, APERATURE_WIDTH, APERATURE_HEIGHT));
    	}
    	for(Point2D.Double coveragePoint : coveragePoints) {
    		for(Path2D.Double rectangle : imageWaypointRectangles) {
    			if(rectangle.contains(coveragePoint)) {
    				covered = true;
    			} 
    		}
	    	if(!covered) {
	    		missedPoints += 1;
	    	}
	    	covered = false;
    	}
    	coverageFraction = 1 - missedPoints/coveragePoints.size();
    	return coverageFraction;
    }
    
    public static double calculateDownstreamRatio(List<Drone> drones) {
    	double downstreamRoutes = 0;
    	double upstreamRoutes = 0;
    	DroneRouteAssignment droneRoute;
    	for(Drone drone : drones) {
    		droneRoute = drone.getDroneRouteAssignment();
    		for(RoutePrimitive route : droneRoute.get()) {
    			if(route.getDownstreamDirection()) {
    				downstreamRoutes += 1;
    			} else {
    				upstreamRoutes += 1;
    			}
    		}
    	}
    	return downstreamRoutes / (downstreamRoutes + upstreamRoutes);
    }
    
    public static boolean batteryFailure(Drone drone) {
    	if(totalDroneDistance(drone) / 1000 > 10) {
    		return true;
    	} else {
    		return false;
    	}
    }
    
	public static int collisionCheck(List<Drone> drones) {
		int collisions = 0;
		for(int i = 0; i < drones.size()-1; i++) {
			List<Point2D.Double> currRoute = drones.get(i).getDroneFullRoute();
			for(int d = i+1; d < drones.size(); d++) {
				List<Point2D.Double> otherRoute = drones.get(d).getDroneFullRoute();
				double currDistance = 0;
				for(int r = 0; r < currRoute.size()-1; r++) {
					Line2D.Double currLineSeg = new Line2D.Double(currRoute.get(r), currRoute.get(r+1));
					double otherDistance = 0;
					for(int r2 = 0; r2 < otherRoute.size()-1; r2++) {
						Line2D.Double otherLineSeg = new Line2D.Double(otherRoute.get(r2), otherRoute.get(r2+1));
						if(currLineSeg.intersectsLine(otherLineSeg)) {
							Point2D.Double intersectionPoint = Geometry.findLineIntersection(currLineSeg, otherLineSeg);
							if(Math.abs((currDistance+Geometry.findCartesianDistance(new Point2D.Double(currLineSeg.getX1(), currLineSeg.getY1()), intersectionPoint))-(otherDistance + Geometry.findCartesianDistance(new Point2D.Double(otherLineSeg.getX1(), otherLineSeg.getY1()), intersectionPoint))) < 10) {
								/*System.out.println(Geometry.findLineIntersection(currLineSeg, otherLineSeg));
								System.out.println("currLineSeg: " + "[" + currLineSeg.getX1() + ", " + currLineSeg.getX2() + "] [" + currLineSeg.getY1() + ", " + currLineSeg.getY2() +"]");
								System.out.println("otherLineSeg: " + "[" + otherLineSeg.getX1() + ", " + otherLineSeg.getX2() + "] [" + otherLineSeg.getY1() + ", " + otherLineSeg.getY2() +"]");
								System.out.println("currDistance: " + currDistance);
								System.out.println("otherDistance: " + otherDistance); */
								collisions += 1;
							}
						}
						otherDistance += Geometry.findCartesianDistance(otherRoute.get(r2), otherRoute.get(r2+1));
					}
					
					currDistance += Geometry.findCartesianDistance(currRoute.get(r), currRoute.get(r+1));
				}
			}
		}
		return collisions;
	}
}
