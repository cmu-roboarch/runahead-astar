/*
 * Some utility functions RA* might use.
 */

#pragma once

#include <algorithm>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <list>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "log.h"

template<typename T>
static void printContainer(T c) {
    for (auto e : c) {
        std::cout << e << " ";
    }
    std::cout << std::endl;
}


template<typename T>
static void printArray(T a, size_t size) {
    for (size_t i = 0; i < size; i++) {
        std::cout << a[i] << " ";
    }
    std::cout << std::endl;
}


template<typename T>
static void printNonIterableContrainer(T c) {
    while (!c.empty()) {
        std::cout << c.top() << " ";
        c.pop();
    }
    std::cout << std::endl;
}


template<typename T>
static std::string containerToString(T c) {
    std::string str = "<";
    for (auto e : c) {
        str += std::to_string(e);
        str += " ";
    }
    str.pop_back();
    str.append(">");
    return str;
}


struct Rectangle {
    /*
     *           length(x)
     *          @─────────┐
     * width(y) │         │
     *          └─────────┘
     * (x, y) is the top-left corner (@)
     */

    int x, y;
    int l, w;

    Rectangle(int _x, int _y, int _l, int _w) {
        x = _x;
        y = _y;
        l = _l;
        w = _w;
    }
};
