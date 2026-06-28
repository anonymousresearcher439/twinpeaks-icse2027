package edu.nd.dronology.services.instances.missionplanning;

import edu.nd.dronology.services.core.api.IFileTransmitServiceInstance;
import edu.nd.dronology.services.core.info.MissionInfo;
import edu.nd.dronology.services.core.util.DronologyServiceException;

public interface IMissionPlanningServiceInstance extends  IFileTransmitServiceInstance<MissionInfo>  {

	void executeMissionPlan(String mission) throws DronologyServiceException;

	void cancelMission() throws DronologyServiceException;

	void removeUAV(String uavid) throws DronologyServiceException;

}
