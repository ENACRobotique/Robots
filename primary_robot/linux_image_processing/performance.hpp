#ifndef PERFORMANCE_H
#define PERFORMANCE_H

#include <chrono>
#include <map>
#include <string>
#include <iostream>

class Perf {
private:
    typedef std::chrono::microseconds time_unit;

    int currFrameNb = 0;

    std::multimap<std::chrono::time_point<std::chrono::high_resolution_clock>, std::string> frameTimes;

    string getPrefix() {
        return "frame #" + std::to_string(currFrameNb) + ": ";
    }

    void insertNow(string s) {
        frameTimes.insert(std::pair<std::chrono::time_point<std::chrono::high_resolution_clock>, std::string>(std::chrono::high_resolution_clock::now(), getPrefix() + s));
    }

public:
    Perf(){

    };

    void beginFrame(int i = -1) {
        currFrameNb = i < 0 ? currFrameNb+1 : i;

        frameTimes.clear();
        insertNow("BEGIN");
    }

    void endOfStep(string s) {
        insertNow("end of step: " + s);
    }

    void endFrame() {
        insertNow("END");

        std::chrono::duration<double> frameDur = std::chrono::duration_cast<time_unit>(frameTimes.crbegin()->first - frameTimes.cbegin()->first);

        multimap<std::chrono::time_point<std::chrono::high_resolution_clock>, std::string>::iterator prevIt;
        for(auto it = frameTimes.begin(); it != frameTimes.end(); it++) {
            if(it != frameTimes.begin()) {
                std::chrono::duration<double> dur = std::chrono::duration_cast<time_unit>(it->first - prevIt->first);

                std::cout << "PERF: " << it->second << " (" << dur.count() << " µs ; " << dur / frameDur * 100. << " %)" << std::endl;
            }
            else {
                std::cout << "PERF: " << it->second << std::endl;
            }

            prevIt = it;
        }
    }
};

#endif  //PERFORMANCE_H
