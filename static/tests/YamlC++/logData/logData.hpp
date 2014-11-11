/*
 * log.hpp
 *
 *  Created on: 10 nov. 2014
 *      Author: Sébastien Malissard
 */

#ifndef LOG_LOG_HPP_
#define LOG_LOG_HPP_

#include <yaml-cpp/emitter.h>
#include <yaml-cpp/parser.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <deque>

#include "../types.hpp"

struct sTrajElLog {
        sTrajEl_t el;
        float date; //date of the message was sending
        unsigned int nbSend; //Number of try to send the trajectory element
};

struct sPathLog {
        sPath_t path;
        std::deque<sTrajElLog> listEl;
};

struct sHeaderLog {
        bool color;
        float version;
        float date; //date of file generated
};

class logData {
    public:
        logData(const std::string file);

        int addPath(const sPath_t newPath);
        int addEl(const unsigned int num, const sTrajEl_t el, const float date, const unsigned int nbSend); //num is the path number
        void addColor(const bool color);

        int getPath(const float time, sPath_t& path, sTrajEl_t& trajEl); //return 1 if a trajectory is available
        bool getColor();

        int save();
        int load();
        void newFile(const std::string file);
        void clean();

    private:
        void headerEmitter(sHeaderLog header, YAML::Emitter& out); //FIXME header surement inutile
        void pathEmitter(std::deque<sPathLog>& path, YAML::Emitter& out);
        void headerParser(YAML::Node& doc);
        void pathParser(YAML::Node& doc);

    private:
        std::deque<sPathLog> listPath;
        sHeaderLog header;
        std::string fileLog;
};

#endif /* LOG_LOG_HPP_ */
