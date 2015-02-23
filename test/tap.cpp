#include "tap.hpp"

#include <iostream>

int Tap::total = 0;
int Tap::errored = 0;
bool Tap::started = false;

void Tap::Start() {
    if (started) {
        return;
    }

    std::cout << "TAP version 13" << std::endl;

    atexit([]() {
        std::cout << std::endl;
        std::cout << "1.." << total << std::endl;
        std::cout << "# tests " << total << std::endl;
        std::cout << "# pass  " << (total - errored) << std::endl;
        std::cout << std::endl;
        if (!errored) {
            std::cout << "# ok" << std::endl;
        } else {
            std::cout << "# not ok" << std::endl;
        }
        std::cout << std::endl;
    });
}

Tap::Test::Test(const std::string &name) {
    std::cout << "# " << name << std::endl;
}

Tap::Test::~Test() {
    if (!finished) {
        fail("test exited without ending");
    }
    if (failed) {
        errored++;
    }
}

void Tap::Test::ok(bool status, const std::string &message) {
    if (!status) {
        fail(message);
    } else {
        std::cout << "ok " << ++total << " " << message << std::endl;
    }
}

void Tap::Test::fail(const std::string &message) {
    failed = true;
    std::cout << "not ok " << ++total << " " << message << std::endl;
}

void Tap::Test::end() {
    finished = true;
}
