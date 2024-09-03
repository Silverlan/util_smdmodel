#ifndef __SMDDEFINITIONS_H__
#define __SMDDEFINITIONS_H__

#ifdef SMDMDL_STATIC
#define DLLSMDMDL
#elif SMDMDL_DLL
#ifdef __linux__
#define DLLSMDMDL __attribute__((visibility("default")))
#else
#define DLLSMDMDL __declspec(dllexport)
#endif
#else
#ifdef __linux__
#define DLLSMDMDL
#else
#define DLLSMDMDL __declspec(dllimport)
#endif
#endif

#endif
