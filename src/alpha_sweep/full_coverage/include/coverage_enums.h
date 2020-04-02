/*-------------------------------------------------------------------------------
 covererage_enums.h
 Version: 1.0
 Commit Date: 07/06/2016

 Authors: Alex von Sternberg
 Los Alamos National Lab

 Description: Enums for the full_coverage package
-------------------------------------------------------------------------------*/

#ifndef _COVERAGE_ENUMS_H_
#define _COVERAGE_ENUMS_H_

namespace full_coverage {

enum CommandMode
{
  CMD_POSE      = 0,
  CMD_VEL       = 1,
  NUM_CMD_MODES = 2
};

} // namespace full_coverage

#endif // _COVERAGE_ENUMS_H_
