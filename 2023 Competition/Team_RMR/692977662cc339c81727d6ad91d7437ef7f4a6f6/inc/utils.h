
#ifndef __UTILS_HH
#define __UTILS_HH

#include <vector>

inline std::pair<int,int> getCoords(int location, int num_col)
{
  return std::pair<int,int>(location / num_col,location % num_col);
}


#endif