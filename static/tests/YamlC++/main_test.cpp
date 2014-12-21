#include "logData/logData.hpp"


sTrajEl_t tabEl1[4] = {
{ { 0., 0. }, { 15.2, -16.9 }, { { 33.2, -16.9 }, 18., 0, 1 }, 0., 0., 0 }, //
{ { 17.3, -8.5 }, { 17.3, -8.5 }, { { -4.9, 3.4 }, -25.2, 0, 1 }, 0., 0., 1 }, //
{ { 17.3, 15.2 }, { 17.3, 15.2 }, { { 33.3, 23.7 }, 18.1, 0, 1 }, 0., 0., 2 }, //
{ { 15.2, 23.7 }, { 15.2, 23.7 }, { { 15.2, 23.7 }, 0., 0, 1 }, 0., 0., 3 } };

sTrajEl_t tabEl2[4] = {
{ { 0., 0. }, { 15.2, -16.9 }, { { 33.2, -16.9 }, 18., 0, 1 }, 0., 0., 0 }, //
{ { 15.2, 23.7 }, { 15.2, 23.7 }, { { 15.2, 23.7 }, 0., 0, 1 }, 0., 0., 3 } };


int main() {

    sPath_t path, path2;
    sTrajEl_t tabEl[10];

    //Create path
    path.dist = 42;
    path.path = tabEl1;
    path.path_len = 4;
    path.tid = 0;

    path2.dist = 0;
    path2.path = tabEl2;
    path2.path_len = 2;
    path2.tid = 1;

    logData obj("log_test.yaml");

    int j = obj.addPath(path);
    for(unsigned int i=0; i < path.path_len; i++){
        obj.addEl(j, path.path[i], 15.,60);
    }

    obj.save();
    j = obj.addPath(path2);
    for(unsigned int i=0; i < path2.path_len; i++){
        obj.addEl(j, path2.path[i], 20., 55);
    }
    obj.save();


    obj.load();

    std::cout << "color" << obj.getColor() << std::endl;

    obj.getPath(100,path,tabEl[1]);
    //obj.getPath(100,path,tabEl[2]);

    std::cout << tabEl[1].sid << std::endl << tabEl[1].p1.x << std::endl << tabEl[1].p2.x << std::endl;
    obj.newFile("test_cpy.yaml");
    obj.save();

    return 0;
}
