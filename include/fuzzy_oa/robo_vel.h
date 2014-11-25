#ifndef HEMU_011014_1700_H
#define HEMU_011014_1700_H

#include <fl/fuzzylite.h>
#include <fl/Engine.h>
#include <fl/variable/OutputVariable.h>
#include <fl/variable/InputVariable.h>
#include <fl/rule/mamdani/MamdaniRule.h>
#include <fl/rule/RuleBlock.h>
#include <fl/term/Trapezoid.h>
#include <fl/term/Rectangle.h>
#include "my_vfh.h" 

#define MAX_LIN_SPEED 2.500
#define MAX_TURNRATE M_PI/4
#define MAX_MAPPING_SPEED 1.500

class vel_control
{
 public:
  vel_control(char* engine_name);
  ~vel_control();

  fl::Engine* engine;
  fl::InputVariable* central_avg;
  fl::InputVariable* dir_avg;
  fl::OutputVariable* angular_vel;
  fl::OutputVariable* linear_vel;
  double lin_vel,ang_vel,lin_vel_limit;
  int proc_start;
  void vel_ctrl_fis(MY_VFH& vfh_obj,double* lin, double* rot,int op_mode);
  void extract_from_vfh(MY_VFH& vfh_obj);
};

#endif
