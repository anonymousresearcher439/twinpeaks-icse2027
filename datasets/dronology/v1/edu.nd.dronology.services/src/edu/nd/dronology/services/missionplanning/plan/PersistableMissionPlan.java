package edu.nd.dronology.services.missionplanning.plan;

import java.util.UUID;

import edu.nd.dronology.services.core.items.IMissionPlan;

public class PersistableMissionPlan implements IMissionPlan{

	private String name;
	private String id;
	private String description;

	
	public PersistableMissionPlan() {
		id = UUID.randomUUID().toString();
		name = id;
	}

	@Override
	public void setName(String name) {
		this.name = name;

	}

	@Override
	public String getId() {
		return id;
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public String getDescription() {
		return description;
	}

	@Override
	public void setDescription(String description) {
		this.description = description;

	}

}
