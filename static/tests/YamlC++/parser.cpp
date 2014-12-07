#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace YAML;

// our data types
struct Vec3 {
    float x, y, z;
};

struct Power {
    std::string name;
    int damage;
};

struct Monster {
    std::string name;
    Vec3 position;
    std::vector<Power> powers;
};

// now the extraction operators for these types
void operator >>(const Node& node, Vec3& v) {
    node[0] >> v.x;
    node[1] >> v.y;
    node[2] >> v.z;
}

void operator >>(const Node& node, Power& power) {
    node["name"] >> power.name;
    node["damage"] >> power.damage;
}

void operator >>(const Node& node, Monster& monster) {
    node["name"] >> monster.name;
    node["position"] >> monster.position;
    const Node& powers = node["powers"];
    for (unsigned i = 0; i < powers.size(); i++) {
        Power power;
        powers[i] >> power;
        monster.powers.push_back(power);
    }
}

int main() {
    std::ifstream fin("monsters.yaml");
    Parser parser(fin);
    Node doc;
    parser.GetNextDocument(doc);
    for (unsigned i = 0; i < doc.size(); i++) {
        Monster monster;
        doc[i] >> monster;
        std::cout << monster.name << "\n";
    }

    return 0;
}
