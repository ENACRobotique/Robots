/**
 * Project Untitled
 */


#ifndef _PTLIDAR_H
#define _PTLIDAR_H

/**
 * \brief      structure Point Lidar.
 */
typedef struct PtLidar {
    int azimut;		/**< Azimut of the point in the robot referential */
    int distance;	/**< Distance from the robot */
    int quality;	/**< Reception quality of the point */
    bool valid;		/**< Is the point valid ? */
    bool warning;	/**< Is there a warnig on the quality of reception ? */
    int updTour;	/**< At wich tour this point has been seen ? */
} PtLidar;

#endif //_PTLIDAR_H
