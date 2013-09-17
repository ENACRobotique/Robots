/*
 * lib_time.h
 *
 *  Created on: 1 mai 2013
 *      Author: quentin
 */

#ifndef LIB_TIME_H_
#define LIB_TIME_H_

extern volatile unsigned long _microsOffset;
extern volatile unsigned long _millisOffset;

inline unsigned long mymicros(){
	return micros()-_microsOffset ;
}

inline unsigned long mymillis(){
	return millis()-_millisOffset ;
}

//local 2 global time reference conversion
inline unsigned long l2gMicros(unsigned long nb){
    return nb-_microsOffset ;
}

inline unsigned long l2gMillis(unsigned long nb){
    return nb-_millisOffset ;
}

void setMicrosOffset(unsigned long offset);
void setMillisOffset(unsigned long offset);


#endif /* LIB_TIME_H_ */
