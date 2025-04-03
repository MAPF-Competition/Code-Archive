#pragma once

#include <vector>

namespace Hungarian{

const int INF = 20000000;
const int FORBID = 10000000;
const int UNWANTED = 100000;

std::vector<int> DoHungarian(const std::vector<std::vector<int>> &a);

bool CheckIfAssigmentCorrect(const std::vector<std::vector<int>> &a, const std::vector<int>& assigment);

}