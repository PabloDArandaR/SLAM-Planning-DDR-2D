#include <math.h>

template <class T>
char sign(T val){
    if (val >= 0) return 1;
    else return -1;
};


template <class T>
void correctAngle(T* error){
    if (abs(error) >= M_PI) *error = sign<T>(error) * (abs(error) % M_PI);
};