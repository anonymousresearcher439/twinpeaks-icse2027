package edu.nd.dronology.services.core.info;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import edu.nd.dronology.services.core.items.TagList;

public class MissionInfo extends RemoteInfoObject {
	/**
	 * 
	 */
	private static final long serialVersionUID = 6569277297683551416L;
	private TagList tags = new TagList();
	private long dateCreated;
	private long dateModified;
	private List<String> uavids;
	private String description;

	public MissionInfo(String name, String id) {
		super(name, id);
		uavids = new ArrayList<>();
	}

	public void setDateModified(long dateModified) {
		this.dateModified = dateModified;
	}

	public void setDateCreated(long dateCreated) {
		this.dateCreated = dateCreated;
	}

	public long getDateCreated() {
		return dateCreated;
	}

	public long getDateModified() {
		return dateModified;
	}

	public String getDescription() {
		return description;
	}

	public void setDescription(String description) {
		this.description = description;
	}

	private void addUavId(String uavid) {
		uavids.add(uavid);
	}

	private Collection<String> getUAVIds() {
		return Collections.unmodifiableCollection(uavids);
	}

	private TagList getTags() {
		return tags;
	}

	private void addTag(String tag) {
		tags.add(tag);
	}
 
}
