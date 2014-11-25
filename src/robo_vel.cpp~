#include "fuzzy_oa/robo_vel.h"
#include <ros/ros.h>

vel_control::vel_control(char* engine_name)
{
  std::string e_name(engine_name);
  engine = new fl::Engine(e_name);
  central_avg = new fl::InputVariable;
  dir_avg = new fl::InputVariable;
  angular_vel = new fl::OutputVariable;
  linear_vel = new fl::OutputVariable;

  /*Denotes mapping mode
  if ( mode == 1)
  MAX_LIN_SPEED = MAX_MAPPING_SPEED;*/
  
  lin_vel_limit = MAX_LIN_SPEED;
  proc_start = 0;

  lin_vel = MAX_LIN_SPEED/2;
  ang_vel = 0.0;

  central_avg->setName("CentralAverage");
  central_avg->setRange(0.000,9.000);
  central_avg->addTerm(new fl::Trapezoid("NEAR",0.000,0.500,1.000,1.50));
  central_avg->addTerm(new fl::Trapezoid("MODERATE",1.000,1.500,4.000,5.000));
  central_avg->addTerm(new fl::Trapezoid("FAR",4.000,5.000,8.000,9.000));
  engine->addInputVariable(central_avg);

  dir_avg->setName("DirectionalAverage");
  dir_avg->setRange(0.000,9.000);
  dir_avg->addTerm(new fl::Trapezoid("NEAR",0.000,0.500,0.700,1.000));
  dir_avg->addTerm(new fl::Trapezoid("MODERATE",0.700,1.000,1.500,2.000));
  dir_avg->addTerm(new fl::Trapezoid("FAR",1.500,2.000,8.000,9.000));
  engine->addInputVariable(dir_avg);

  angular_vel->setName("AngularVel");
  angular_vel->setRange(0.000,MAX_TURNRATE);
  angular_vel->setDefaultValue(0);
  angular_vel->addTerm(new fl::Rectangle("LOW",0.000,MAX_TURNRATE/5));
  angular_vel->addTerm(new fl::Rectangle("MEDIUM",MAX_TURNRATE/5,3*MAX_TURNRATE/5));
  angular_vel->addTerm(new fl::Rectangle("HIGH",3*MAX_TURNRATE/5,MAX_TURNRATE));
  engine->addOutputVariable(angular_vel);

  linear_vel->setName("LinearVel");
  linear_vel->setRange(0.000,lin_vel_limit);
  linear_vel->setDefaultValue(0);
  linear_vel->addTerm(new fl::Rectangle("SLOW",0.000,lin_vel_limit/5));
  linear_vel->addTerm(new fl::Rectangle("MEDIUM",lin_vel_limit/5,3*lin_vel_limit/5));
  linear_vel->addTerm(new fl::Rectangle("FAST",3*lin_vel_limit/5,lin_vel_limit));
  engine->addOutputVariable(linear_vel);

  fl::RuleBlock* ruleblock1=new fl::RuleBlock;
  ruleblock1->addRule(fl::MamdaniRule::parse("if DirectionalAverage is NEAR and CentralAverage is NEAR then AngularVel is HIGH", engine));
  ruleblock1->addRule(fl::MamdaniRule::parse("if DirectionalAverage is NEAR and CentralAverage is MODERATE then AngularVel is MEDIUM", engine));
  ruleblock1->addRule(fl::MamdaniRule::parse("if DirectionalAverage is NEAR and CentralAverage is FAR then AngularVel is MEDIUM", engine));
  ruleblock1->addRule(fl::MamdaniRule::parse("if DirectionalAverage is MODERATE and CentralAverage is NEAR then AngularVel is MEDIUM",engine));
  ruleblock1->addRule(fl::MamdaniRule::parse("if DirectionalAverage is MODERATE and CentralAverage is MODERATE then AngularVel is MEDIUM",engine));
  ruleblock1->addRule(fl::MamdaniRule::parse("if DirectionalAverage is MODERATE and CentralAverage is FAR then AngularVel is LOW",engine));
  ruleblock1->addRule(fl::MamdaniRule::parse("if DirectionalAverage is FAR and CentralAverage is FAR then AngularVel is LOW", engine));
  ruleblock1->addRule(fl::MamdaniRule::parse("if DirectionalAverage is FAR and CentralAverage is MODERATE then AngularVel is LOW", engine));
  engine->addRuleBlock(ruleblock1);

  fl::RuleBlock* ruleblock2=new fl::RuleBlock;
  ruleblock2->addRule(fl::MamdaniRule::parse("if CentralAverage is NEAR then LinearVel is SLOW", engine));
  ruleblock2->addRule(fl::MamdaniRule::parse("if CentralAverage is MODERATE and DirectionalAverage is FAR then LinearVel is MEDIUM",engine));
  ruleblock2->addRule(fl::MamdaniRule::parse("if CentralAverage is MODERATE and DirectionalAverage is MODERATE then LinearVel is MEDIUM",engine));
  ruleblock2->addRule(fl::MamdaniRule::parse("if CentralAverage is MODERATE and DirectionalAverage is NEAR then LinearVel is SLOW",engine));
  ruleblock2->addRule(fl::MamdaniRule::parse("if CentralAverage is FAR and DirectionalAverage is NEAR then LinearVel is MEDIUM", engine));
  ruleblock2->addRule(fl::MamdaniRule::parse("if CentralAverage is FAR and DirectionalAverage is MODERATE then LinearVel is FAST",engine));
  ruleblock2->addRule(fl::MamdaniRule::parse("if CentralAverage is FAR and DirectionalAverage is FAR then LinearVel is FAST",engine));
  engine->addRuleBlock(ruleblock2);

  engine->configure("Minimum", "Maximum", "AlgebraicProduct", "AlgebraicSum", "Centroid");
}

vel_control::~vel_control()
{

}

void vel_control::extract_from_vfh(MY_VFH& vfh_obj)
{
  float out1=0.0;  //linear_speed                       
  float out2=0.0;  //rotational_speed

  if (vfh_obj.highres_vfh.empty() && vfh_obj.medres_vfh.empty() && vfh_obj.lowres_vfh.empty())
    {
      out1 = lin_vel;
      out2 = ang_vel;
    }

  else
    {
      double highres_center = vfh_obj.highres_vfh[HIGHRES_SEC/2];
      double medres_center = vfh_obj.medres_vfh[MEDRES_SEC/2];
      double medres_left=0.0, medres_right=0.0,medres_sright=0.0,medres_sleft=0.0;

      for ( int ii=0; ii < (MEDRES_SEC/2); ii++ )
        {
          if ( ii == (MEDRES_SEC/2)-1 )
            {
              medres_sright = vfh_obj.medres_vfh[ii];
              medres_sleft = vfh_obj.medres_vfh[MEDRES_SEC-1-ii];
            }
          medres_right+=vfh_obj.medres_vfh[ii];
          medres_left+=vfh_obj.medres_vfh[MEDRES_SEC-1-ii];
        }

      medres_left/=2;
      medres_right/=2;

      //ROS_INFO("%f %f %f %f %f", medres_left, medres_sleft, medres_center, medres_sright, medres_right);

      fl::scalar in1;
      fl::scalar in2(0.0);
      if ( medres_center > highres_center )
	in1 = highres_center;
      else
	in1 = medres_center;
      
      double aR, aL;      

      if ( in1 < 1.0 )
	{
	  if ( medres_sright < medres_sleft )
	    {
	      in2 = medres_sright;
	      aL = 0.0;
	      aR = in2;
	    }
	  else if ( medres_sleft < medres_sright )
	    {
	      in2 = medres_sleft;
	      aR = 0.0;
	      aL = in2;
	    }
	  else 
	    {
	      in2 = medres_sright;
	      aL = 0.0;
	      aR = in2;
	    }
	}

      else
	{
	  if ( medres_sright < medres_right)
	    aR = medres_sright;
	  else 
	    aR = medres_right;

	  if ( medres_sleft < medres_left)
            aL = medres_sleft;
          else
            aL = medres_left;

	  if ( aR > aL+0.25 )
	    {
	      in2 = aL;
	      aR = 0.0;
	    }
	  else if ( aL > aR+0.25 )
	    {
	      in2 = aR;
	      aL = 0.0;
	    }
	  else
	    {
	      in2 = aL;
	      aR = 0.0;
	    }
	}
      
      central_avg->setInput(in1);
      dir_avg->setInput(in2);
      engine->process();
      out1 = linear_vel->defuzzify();
      out2 = angular_vel->defuzzify();
      
      if ( in1 < 0.5 )
	out1 = 0.0;
      else if ( in2 > 1.0 && in1 > 1.0 )
        out2 = 0.0;

      if ( aR == 0.0 )
	out2*=(-1.0);
      else if ( aL == 0.0 )
        out2*=1.0;

      vfh_obj.highres_vfh.clear();
      vfh_obj.medres_vfh.clear();
      vfh_obj.lowres_vfh.clear();
    }

  lin_vel = out1;
  ang_vel = out2;
}

void vel_control::vel_ctrl_fis(MY_VFH& vfh_obj, double* lin, double* rot, int op_mode)
{
  //Denotes mapping mode                                           
    if ( op_mode == 1 && proc_start == 0)
      {
	lin_vel_limit = MAX_MAPPING_SPEED;
	proc_start = 1;
      }
    else if ( op_mode == 0 && proc_start == 0 )
      {
	proc_start = 1;
      }

  extract_from_vfh(vfh_obj);

  *lin=lin_vel;
  *rot=ang_vel;

}
