/*
 * distinguish.h
 *
 *  Created on: 25 févr. 2016
 *      Author: fabien
 */

#ifndef SRC_DISTINGUISH_H_
#define SRC_DISTINGUISH_H_

#include <vector>
#include "Group.h"
#include "data_table.h"
#include <algorithm>
using namespace std;

vector<Group> select_groups_by_size(vector<Group> groups, int object_size);
vector<Group> select_groups_by_distance(vector<Group> groups, int dmin, int dmax);



#endif /* SRC_DISTINGUISH_H_ */
