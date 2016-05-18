#ifndef UTILITY_H
#define UTILITY_H

#include <string>
#include <vector>
#include <sstream>

inline std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

inline std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

inline void remove_quotes(std::string &s) {
    if (s.substr(s.length()-1, 1).compare("\"") == 0) {
        s.erase(s.length()-1, 1);
    }
    if (s.substr(0, 1).compare("\"") == 0) {
        s.erase(0, 1);
    }    
}

#endif // UTILITY_H