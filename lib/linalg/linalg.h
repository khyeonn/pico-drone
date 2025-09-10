#ifndef LINALG_H
#define LINALG_H



void set_diag3(float M[3][3], const float v[3]);
// Fill a 6x6 diagonal matrix from [va, vm] (values squared)
void set_diag6(float M[6][6], const float va[3], const float vm[3]);



#endif // LINALG_H