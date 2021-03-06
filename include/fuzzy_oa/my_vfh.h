#ifndef VFH_011014_1703
#define VFH_011014_1703

#include <vector>

#define HIGHRES_SEC 15
#define MEDRES_SEC 5
#define LOWRES_SEC 3

typedef struct {
  std::vector<double> highres_vfh;
  std::vector<double> medres_vfh;
  std::vector<double> lowres_vfh; 
}MY_VFH;
#endif 
