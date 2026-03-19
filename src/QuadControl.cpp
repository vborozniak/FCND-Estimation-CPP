#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float c_bar = collThrustCmd / 4.f;
  float p_bar = momentCmd.x / (4.f * L);
  float q_bar = momentCmd.y / (4.f * L);
  float r_bar = momentCmd.z / (4.f * kappa);

  cmd.desiredThrustsN[0] = c_bar + p_bar + q_bar - r_bar; // front left
  cmd.desiredThrustsN[1] = c_bar - p_bar + q_bar + r_bar; // front right
  cmd.desiredThrustsN[2] = c_bar + p_bar - q_bar + r_bar; // rear left
  cmd.desiredThrustsN[3] = c_bar - p_bar - q_bar - r_bar; // rear right

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////


  momentCmd.x = Ixx * kpPQR.x * (pqrCmd.x - pqr.x);
  momentCmd.y = Iyy * kpPQR.y * (pqrCmd.y - pqr.y);
  momentCmd.z = Izz * kpPQR.z * (pqrCmd.z - pqr.z);
  

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  pqrCmd.z = 0.f;

  float c = collThrustCmd / mass;

  if (c > 0.0f) {
    float target_R13 = -accelCmd.x / c;
    float target_R23 = -accelCmd.y / c;

    float horiz_proj = sqrtf(target_R13*target_R13 + target_R23*target_R23);
    if (horiz_proj > sinf(maxTiltAngle) && horiz_proj > 0.001f) {
      target_R13 *= sinf(maxTiltAngle) / horiz_proj;
      target_R23 *= sinf(maxTiltAngle) / horiz_proj;
    }

    float err_R13 = target_R13 - R(0,2);
    float err_R23 = target_R23 - R(1,2);

    // Add scaled gain — softens at tilt for smooth deceleration, reduces overshoot
    pqrCmd.y = kpBank * err_R13 / R(2,2);      // q_cmd
    pqrCmd.x = -kpBank * err_R23 / R(2,2);     // p_cmd (negative cross-coupling)
  }

  return pqrCmd;
}
float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
 
  float g = 9.81f;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  integratedAltitudeError += (posZCmd - posZ) * dt;

  float posError = posZCmd - posZ;
  float velError = velZCmd - velZ;

  float desiredAccelZ = kpPosZ * posError + kpVelZ * velError + KiPosZ * integratedAltitudeError + accelZCmd;

  float thrust_raw = mass * (g + desiredAccelZ) / R(2,2);

  float thrust = CONSTRAIN(thrust_raw, minMotorThrust*4.f, maxMotorThrust*4.f);
  if (thrust != thrust_raw) {
    integratedAltitudeError -= (posZCmd - posZ) * dt;  // Back-off
  }
  

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  accelCmdFF.z = 0.f;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;



  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  V3F posError = posCmd - pos;

  // Outer loop: Generate desired velocity from position error + commanded velocity
  V3F desVel = velCmd;
  desVel.x += kpPosXY * posError.x;
  desVel.y += kpPosXY * posError.y;

  // Limit desired horizontal velocity magnitude
  float desVelHorizMag = sqrtf(desVel.x * desVel.x + desVel.y * desVel.y);
  if (desVelHorizMag > maxSpeedXY && desVelHorizMag > 0.001f) {  // Avoid divide-by-zero
    desVel.x *= maxSpeedXY / desVelHorizMag;
    desVel.y *= maxSpeedXY / desVelHorizMag;
  }

  // Inner loop: Velocity PD to generate acceleration
  V3F velError = desVel - vel;
  accelCmd.x += kpVelXY * velError.x;
  accelCmd.y += kpVelXY * velError.y;

  // Limit horizontal acceleration magnitude (your existing limit, retained)
  float horizMag = sqrtf(accelCmd.x * accelCmd.x + accelCmd.y * accelCmd.y);
  if (horizMag > maxAccelXY && horizMag > 0.001f) {
    accelCmd.x *= maxAccelXY / horizMag;
    accelCmd.y *= maxAccelXY / horizMag;
  }

  accelCmd.z = 0.f;

  /////////////////////////////// END STUDENT CODE ////////////////////////////


  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float yawError = yawCmd - yaw;

// unwrap to [-π, π]
  yawError = fmodf(yawError + 3.f * F_PI, 2.f * F_PI) - F_PI;

  yawRateCmd = kpYaw * yawError;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  //float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  //collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
