//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 04-Oct-2024 17:20:29
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include Files
#include "main.h"
#include "forward_dynamics.h"
#include "forward_dynamics_terminate.h"
#include "forward_dynamics_types.h"
#include <algorithm>

// Function Declarations
static void argInit_14x1_real_T(double result[14]);

static void argInit_15x1_real_T(double result[15]);

static void argInit_1x3_real_T(double result[3]);

static void argInit_1x4_real_T(double result[4]);

static void argInit_1x6_real_T(double result[6]);

static void argInit_3x3_real_T(double result[9]);

static void argInit_4x4_real_T(double result[16]);

static void argInit_5x1_real_T(double result[5]);

static void argInit_6x6_real_T(double result[36]);

static void argInit_6x7_real_T(double result[42]);

static double argInit_real_T();

static void argInit_struct0_T(struct0_T &result);

static void argInit_struct1_T(struct1_T &result);

static void argInit_struct2_T(struct2_T &result);

static void argInit_struct3_T(struct3_T &result);

static void argInit_struct4_T(struct4_T &result);

static void argInit_struct5_T(struct5_T &result);

static void argInit_struct6_T(struct6_T &result);

static void argInit_struct7_T(struct7_T &result);

// Function Definitions
//
// Arguments    : double result[14]
// Return Type  : void
//
static void argInit_14x1_real_T(double result[14])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 14; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[15]
// Return Type  : void
//
static void argInit_15x1_real_T(double result[15])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 15; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[3]
// Return Type  : void
//
static void argInit_1x3_real_T(double result[3])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 3; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
}

//
// Arguments    : double result[4]
// Return Type  : void
//
static void argInit_1x4_real_T(double result[4])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 4; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
}

//
// Arguments    : double result[6]
// Return Type  : void
//
static void argInit_1x6_real_T(double result[6])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 6; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
}

//
// Arguments    : double result[9]
// Return Type  : void
//
static void argInit_3x3_real_T(double result[9])
{
  // Loop over the array to initialize each element.
  for (int i{0}; i < 9; i++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[i] = argInit_real_T();
  }
}

//
// Arguments    : double result[16]
// Return Type  : void
//
static void argInit_4x4_real_T(double result[16])
{
  // Loop over the array to initialize each element.
  for (int i{0}; i < 16; i++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[i] = argInit_real_T();
  }
}

//
// Arguments    : double result[5]
// Return Type  : void
//
static void argInit_5x1_real_T(double result[5])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 5; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[36]
// Return Type  : void
//
static void argInit_6x6_real_T(double result[36])
{
  // Loop over the array to initialize each element.
  for (int i{0}; i < 36; i++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[i] = argInit_real_T();
  }
}

//
// Arguments    : double result[42]
// Return Type  : void
//
static void argInit_6x7_real_T(double result[42])
{
  // Loop over the array to initialize each element.
  for (int i{0}; i < 42; i++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[i] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : struct0_T &result
// Return Type  : void
//
static void argInit_struct0_T(struct0_T &result)
{
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_1x6_real_T(result.Pose);
  result_tmp = argInit_real_T();
  result.NumThrusters = result_tmp;
  result.Volume = result_tmp;
  argInit_1x3_real_T(result.CenterOfMass);
  argInit_6x6_real_T(result.AddedMassMatrix);
  result.NumDoFs = result_tmp;
  result.g = result_tmp;
  result.rho = result_tmp;
  result.Mass = result_tmp;
  argInit_3x3_real_T(result.Inertia);
  argInit_6x7_real_T(result.ThrusterAllocationMatrix);
  argInit_struct1_T(result.Arms);
  for (int i{0}; i < 6; i++) {
    result.QuasiVel[i] = result.Pose[i];
  }
  result.CenterOfBuoyancy[0] = result.CenterOfMass[0];
  result.CenterOfBuoyancy[1] = result.CenterOfMass[1];
  result.CenterOfBuoyancy[2] = result.CenterOfMass[2];
  for (int i{0}; i < 36; i++) {
    result.LinearDamping[i] = result.AddedMassMatrix[i];
    result.QuadraticDamping[i] = result.AddedMassMatrix[i];
  }
}

//
// Arguments    : struct1_T &result
// Return Type  : void
//
static void argInit_struct1_T(struct1_T &result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_struct2_T(result.Arm1);
  result.Arm2 = result.Arm1;
}

//
// Arguments    : struct2_T &result
// Return Type  : void
//
static void argInit_struct2_T(struct2_T &result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_4x4_real_T(result.TransformVehicleToArmBase);
  argInit_1x4_real_T(result.JointVariables);
  argInit_struct3_T(result.DHParameters);
  result.NumDofs = argInit_real_T();
  argInit_struct4_T(result.Links);
  argInit_struct6_T(result.Joints);
  std::copy(&result.TransformVehicleToArmBase[0],
            &result.TransformVehicleToArmBase[16], &result.LastLinkToEE[0]);
  result.JointVelocities[0] = result.JointVariables[0];
  result.ZeroPosition[0] = result.JointVariables[0];
  result.JointVelocities[1] = result.JointVariables[1];
  result.ZeroPosition[1] = result.JointVariables[1];
  result.JointVelocities[2] = result.JointVariables[2];
  result.ZeroPosition[2] = result.JointVariables[2];
  result.JointVelocities[3] = result.JointVariables[3];
  result.ZeroPosition[3] = result.JointVariables[3];
}

//
// Arguments    : struct3_T &result
// Return Type  : void
//
static void argInit_struct3_T(struct3_T &result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_5x1_real_T(result.d);
  for (int i{0}; i < 5; i++) {
    result.theta[i] = result.d[i];
    result.a[i] = result.d[i];
    result.alpha[i] = result.d[i];
  }
}

//
// Arguments    : struct4_T &result
// Return Type  : void
//
static void argInit_struct4_T(struct4_T &result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_struct5_T(result.Link1);
  result.Link2 = result.Link1;
  result.Link3 = result.Link1;
  result.Link4 = result.Link1;
  result.Link5 = result.Link1;
}

//
// Arguments    : struct5_T &result
// Return Type  : void
//
static void argInit_struct5_T(struct5_T &result)
{
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result.Volume = result_tmp;
  argInit_1x3_real_T(result.CenterOfMass);
  argInit_6x6_real_T(result.AddedMass);
  result.Mass = result_tmp;
  argInit_3x3_real_T(result.InertiaTensor);
  result.CenterOfBuoyancy[0] = result.CenterOfMass[0];
  result.CenterOfBuoyancy[1] = result.CenterOfMass[1];
  result.CenterOfBuoyancy[2] = result.CenterOfMass[2];
  for (int i{0}; i < 36; i++) {
    result.LinearDamping[i] = result.AddedMass[i];
    result.QuadraticDamping[i] = result.AddedMass[i];
  }
}

//
// Arguments    : struct6_T &result
// Return Type  : void
//
static void argInit_struct6_T(struct6_T &result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_struct7_T(result.Joint1);
  result.Joint2 = result.Joint1;
  result.Joint3 = result.Joint1;
  result.Joint4 = result.Joint1;
}

//
// Arguments    : struct7_T &result
// Return Type  : void
//
static void argInit_struct7_T(struct7_T &result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_4x4_real_T(result.TransformParentToChild);
}

//
// Arguments    : int argc
//                char **argv
// Return Type  : int
//
int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_forward_dynamics();
  // Terminate the application.
  // You do not need to do this more than one time.
  forward_dynamics_terminate();
  return 0;
}

//
// Arguments    : void
// Return Type  : void
//
void main_forward_dynamics()
{
  struct0_T r;
  double dv[15];
  double dksi_dt[14];
  double dzeta_dt_data[14];
  double ksi_tmp[14];
  int dzeta_dt_size;
  // Initialize function 'forward_dynamics' input arguments.
  // Initialize function input argument 'params'.
  // Initialize function input argument 'ksi'.
  argInit_14x1_real_T(ksi_tmp);
  // Initialize function input argument 'zeta'.
  // Initialize function input argument 'control_input'.
  // Call the entry-point 'forward_dynamics'.
  argInit_struct0_T(r);
  argInit_15x1_real_T(dv);
  forward_dynamics(&r, ksi_tmp, ksi_tmp, dv, dksi_dt, dzeta_dt_data,
                   *(int(*)[1]) & dzeta_dt_size);
}

//
// File trailer for main.cpp
//
// [EOF]
//