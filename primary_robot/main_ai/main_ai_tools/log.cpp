/*
 * log.cpp
 *
 *  Created on: 1 mars 2015
 *      Author: seb
 */

#include "log.h"

#include <ctime>
#include <string.h>
#include <cmath>
#include <iomanip>



Log::Log(): _file("log.log"), _type(E_OTHER), _ver(E_V2), _pos(false){
}

Log::Log(char* file): _file(file), _type(E_OTHER), _ver(E_V2), _pos(false){
}

Log::~Log() {
    // TODO Auto-generated destructor stub
}

/*
 * Changes the file log.
 */
void Log::changeFile(const char* file){
     _file.close();
     _file.open(file);
}

void Log::setType(const eLog_t &log){
    _type = log;
}

void Log::setVer(const eVer_t &ver){
    _ver = ver;

    putNewMes();
}

/*
 * Writes the beginning of the message
 */
void Log::putNewMes(){
    time_t now;
    char *date;
    size_t length;

    time(&now);
    date = ctime(&now);

    if ((length = strlen(date)) > 0)
        date[length-1] = ' ';

    _file << endl << date;
    if(term()){
        if(_pos)
            cout << "\x0d\033[K";
        else
            cout << endl;
        _pos = false;
    }
}

void Log::putNewPos(float x, float y, float theta){
    if(!_pos)
        cout << endl;

    _pos = true;

    cout << fixed << setprecision(2) << "\x1b[K\x1b[s" << "pos : " << x << "cm, " << y << "cm, " << theta * 180. / M_PI << "°" << "\x1b[u" << flush;

}

/*
 * Writes a string format in char*
 */
void Log::putText(const char* text){
    _file << text;
    if(term())
        cout << text;
}

/*
 * Writes a float
 */
void Log::putNum(const float& num){
    _file << num;
    if(term())
        cout << num;
}

/*
 * Return true if the message must be writes to the display.
 */
bool Log::term() const{
    if(_ver <= E_V2)
        return true;
    return false;
}


Log& operator<<(Log& log, const char*text){
    log.putText(text);
    return log;
}

Log& operator<<(Log& log, const int num){
    log.putNum((float) num);
    return log;
}

Log& operator<<(Log& log, eLog_t type){
    log.setType(type);
    return log;
}

Log& operator<<(Log& log, eVer_t ver){
    log.setVer(ver);
    return log;
}

