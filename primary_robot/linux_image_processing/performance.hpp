#ifndef PERFORMANCE_H
#define PERFORMANCE_H

#include <chrono>
#include <map>
#include <string>
#include <iostream>

class Perf {
private:
    using duration = std::chrono::duration<double, std::milli>;
    using seconds = std::chrono::duration<double>;
    using time_point = std::chrono::time_point<std::chrono::high_resolution_clock>;

    int currFrameNb = 0;

    std::multimap<time_point, std::string> frameTimes;

    std::string getPrefix() {
        return "frame #" + std::to_string(currFrameNb) + ": ";
    }

    void insertNow(std::string s) {
        frameTimes.insert(std::make_pair(time_point::clock::now(), getPrefix() + s));
    }

    Perf() {
    }

public:
    static Perf& getPerf() {
        static Perf p;
        return p;
    }

    void beginFrame(int i = -1) {
        currFrameNb = i < 0 ? currFrameNb + 1 : i;

        frameTimes.clear();
        insertNow("BEGIN");
    }

    void endOfStep(std::string s) {
        insertNow("STEP  " + s);
    }

    void endFrame() {
        insertNow("END  ");

        duration frameDur = std::chrono::duration_cast<duration>(frameTimes.crbegin()->first - frameTimes.cbegin()->first);

        std::multimap<time_point, std::string>::iterator prevIt;
        for (auto it = frameTimes.begin(); it != frameTimes.end(); it++) {
            if (it != frameTimes.begin()) {
                duration dur = std::chrono::duration_cast<duration>(it->first - prevIt->first);

                std::cout << "PERF: " << it->second << " (" << dur.count() << " ms ; " << dur / frameDur * 100. << " %)" << std::endl;
            }
            else {
                std::cout << "PERF: " << it->second << " (total: " << frameDur.count() << " ms ; " << 1. / std::chrono::duration_cast<seconds>(frameDur).count() << " FPS)" << std::endl;
            }

            prevIt = it;
        }
    }
};

#endif  //PERFORMANCE_H
