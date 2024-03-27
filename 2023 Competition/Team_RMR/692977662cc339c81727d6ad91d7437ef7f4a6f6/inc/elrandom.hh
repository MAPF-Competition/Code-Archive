#ifndef __ELRANDOM_HH
#define __ELRANDOM_HH

#include <random>
#include <iostream>

//EasyLocal Random (elrandom.hh)

class ElRandom
{
public:
  /** Generates an uniform random integer in [a, b].
   @param a lower bound
    @param b upper bound
    */
  template <typename T, typename std::enable_if<std::is_integral<T>::value>::type* = nullptr>
  static T Uniform(T a, T b)
  {
    std::uniform_int_distribution<T> d(a, b);
    return d(GetInstance().g);
  }
  
  /** Generates an uniform random float in [a, b].
   @param a lower bound
    @param b upper bound
    */
  template <typename T, typename std::enable_if<std::is_floating_point<T>::value>::type* = nullptr>
  static T Uniform(T a, T b)
  {
    std::uniform_real_distribution<T> d(a, b);
    return d(GetInstance().g);
  }
  
  /** Sets a new seed for the random engine. */
  static unsigned int SetSeed(unsigned int seed)
  {
    ElRandom& r = GetInstance();
    r.g.seed(seed);
    return r.seed = seed;
  }
  
  static unsigned int GetSeed()
  {
    return GetInstance().seed;
  }
  
  
  static std::mt19937& GetGenerator()
  {
    return GetInstance().g;
  }
  
private:
  static ElRandom& GetInstance() {
    static ElRandom instance;
    return instance;
  }
  
  ElRandom()
  {
    std::random_device dev;
    seed = dev();
    g.seed(seed);
  }
  
  std::mt19937 g;
  
  unsigned int seed;
};

#endif