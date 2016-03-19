/*
 * data.h
 *
 *  Created on: 20 févr. 2016
 *      Author: fabien
 */

#ifndef SRC_DATA_TABLE_H_
#define SRC_DATA_TABLE_H_

#include "vector"
#include "math.h"
using namespace std;

typedef struct object {
	int x;
	int y;
	int size;
	const char * nom;
	inline bool operator==(const object &other)
	{
	    if((x == other.x) && (y == other.y) && (size == other.size) ){
	    	return true;
	    }
	    return false;
	}

	inline bool operator!=(const object &other)
	{
		return !operator==(other);
	}
} object;

void initObjs();
int objDist(object obj1, object obj2);
vector <object> getObjs();

extern vector <object> tableMap;

#endif /* SRC_DATA_TABLE_H_ */
