/*-------------------------------------------------------------------------------
 sweep_gui_enums.h
 Version: 1.0
 Date: 09/27/2016

 Authors: Alex von Sternberg
 Los Alamos National Lab

 Description: Enums for the sweep_gui package
-------------------------------------------------------------------------------*/

#ifndef _SWEEP_GUI_ENUMS_H_
#define _SWEEP_GUI_ENUMS_H_

namespace sweep_gui {

enum Status
{
  STATUS_INITIALIZING = 0,
  STATUS_IDLE         = 1,
  STATUS_PLANNING     = 2,
  STATUS_EXECUTING    = 3,
  STATUS_PAUSED       = 4,
  STATUS_UNCLEARED    = 5,
  STATUS_JOY          = 6,
  STATUS_ESTOPPED     = 7,
  STATUS_DOCKING      = 8,
  STATUS_DOCKED       = 9,
  STATUS_UNDOCKING    = 10,
  NUM_STATUS          = 11
};

} // namespace sweep_gui

#endif // _SWEEP_GUI_ENUMS_H_
