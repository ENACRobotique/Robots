/*
 * distinguish.cpp
 *
 *  Created on: 25 févr. 2016
 *      Author: fabien
 */
#include "distinguish.h"


/*
 * Select groups of size about object_size. (maxSize >= "object_size" && minSize <= "object_size")
 */
vector<Group> select_groups_by_size(vector<Group> groups, int object_size){
	vector<Group> potentials_groups;
	for (Group grp : groups) {
		int maxSize = grp.getMaxSize();
		int minSize = grp.getMinSize();
		if(minSize <= object_size && maxSize >= object_size){			//size < 0 mean there is an error in the size calculation.
			potentials_groups.push_back(grp);
		}
	}
	return potentials_groups;
}

vector<Group> select_groups_by_distance(vector<Group> groups, int dmin, int dmax){
	vector<Group> potentials_groups;
	for (Group grp : groups) {
		int dist = grp.getDist();
		if(dist > dmin && dist < dmax){			//size < 0 mean there is an error in the size calculation.
			potentials_groups.push_back(grp);
		}
	}
	return potentials_groups;
}
