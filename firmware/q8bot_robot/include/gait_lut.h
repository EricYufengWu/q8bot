#ifndef GAIT_LUT_H
#define GAIT_LUT_H


#if __has_include("_gait_lut_generated.h")
    #define __INCLUDING_GENERATED_LUTS
    #include "_gait_lut_generated.h"
    #undef __INCLUDING_GENERATED_LUTS
#else
    #error "ERROR: Please generate gait LUTs by running the python-tools/q8bot/gait_lut.py script!"
#endif // #if __has_include("_gait_lut_generated.h")


#endif // #ifndef GAIT_LUT_H