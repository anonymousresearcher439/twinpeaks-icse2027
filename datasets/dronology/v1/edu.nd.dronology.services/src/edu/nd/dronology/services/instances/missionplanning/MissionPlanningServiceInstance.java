package edu.nd.dronology.services.instances.missionplanning;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.attribute.BasicFileAttributes;

import edu.nd.dronology.services.core.api.IFileChangeNotifyable;
import edu.nd.dronology.services.core.base.AbstractFileTransmitServiceInstance;
import edu.nd.dronology.services.core.info.MissionInfo;
import edu.nd.dronology.services.core.items.IMissionPlan;
import edu.nd.dronology.services.core.persistence.MissionPlanningPersistenceProvider;
import edu.nd.dronology.services.core.persistence.PersistenceException;
import edu.nd.dronology.services.core.util.DronologyConstants;
import edu.nd.dronology.services.core.util.DronologyServiceException;
import edu.nd.dronology.services.core.util.ServiceIds;
import edu.nd.dronology.services.instances.DronologyElementFactory;
import edu.nd.dronology.services.missionplanning.MissionExecutionException;
import edu.nd.dronology.services.missionplanning.plan.MissionController;
import edu.nd.dronology.services.missionplanning.sync.SynchronizationManager;
import edu.nd.dronology.services.supervisor.SupervisorService;
import edu.nd.dronology.util.FileUtil;
import net.mv.logging.ILogger;
import net.mv.logging.LoggerProvider;

public class MissionPlanningServiceInstance extends AbstractFileTransmitServiceInstance<MissionInfo>
		implements IFileChangeNotifyable, IMissionPlanningServiceInstance {

	private static final ILogger LOGGER = LoggerProvider.getLogger(MissionPlanningServiceInstance.class);

	public static final String EXTENSION = DronologyConstants.EXTENSION_MISSION;

	public MissionPlanningServiceInstance() {
		super(ServiceIds.SERVICE_MISSIONPLANNING, "Mission Planning",EXTENSION);
	}

	@Override
	protected Class<?> getServiceClass() {
		return MissionPlanningService.class;
	}

	@Override
	protected int getOrder() {
		// TODO Auto-generated method stub
		return 2;
	}

	@Override
	protected String getPropertyPath() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected void doStartService() throws Exception {

	}

	@Override
	protected void doStopService() throws Exception {
		// TODO Auto-generated method stub

	}

	@Override
	public void executeMissionPlan(String MissionP) throws DronologyServiceException {
//		try {
//			MissionController.getInstance().executeMission(mission);
//		} catch (MissionExecutionException e) {
//			LOGGER.error(e);
//			new DronologyServiceException(e.getMessage());
//		}

	}

	@Override
	public void cancelMission() throws DronologyServiceException {
		try {
			MissionController.getInstance().cancelMission();
		} catch (MissionExecutionException e) {
			LOGGER.error(e);
			new DronologyServiceException(e.getMessage());
		}

	}

	@Override
	public void removeUAV(String uavid) throws DronologyServiceException {
		SynchronizationManager.getInstance().removeUAV(uavid);

	}

	@Override
	public MissionInfo createItem() throws DronologyServiceException {
		MissionPlanningPersistenceProvider persistor = MissionPlanningPersistenceProvider.getInstance();
		IMissionPlan missionPlan = DronologyElementFactory.createNewMissionPlan();
		missionPlan.setName("New-MissionPlan");
		String savePath = FileUtil.concat(storagePath, missionPlan.getId(), EXTENSION);

		try {
			persistor.saveItem(missionPlan, savePath);
		} catch (PersistenceException e) {
			throw new DronologyServiceException("Error when creating mission plan: " + e.getMessage());
		}
		return new MissionInfo(missionPlan.getName(), missionPlan.getId());
	}

	@Override
	protected String getPath() {
		String path = SupervisorService.getInstance().getMissionPlanningLocation();
		return path;
	}

	@Override
	protected MissionInfo fromFile(String id, File file) throws Throwable {
		IMissionPlan atm = MissionPlanningPersistenceProvider.getInstance().loadItem(file.toURI().toURL());
		MissionInfo info = new MissionInfo(atm.getName(), id);
		

		BasicFileAttributes attr = Files.readAttributes(Paths.get(file.toURI()), BasicFileAttributes.class);
//		info.setDateCreated(attr.creationTime().toMillis());
//		info.setDateModified(attr.lastModifiedTime().toMillis());
		info.setDescription(atm.getDescription());
		return info;
	} 

}
