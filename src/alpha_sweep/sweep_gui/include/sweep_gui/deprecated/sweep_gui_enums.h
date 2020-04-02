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
  STATUS_MAP_SWEEP    = 2,
  STATUS_PLANNING     = 3,
  STATUS_READY        = 4,
  STATUS_EXECUTING    = 5,
  STATUS_PAUSED       = 6,
  STATUS_UNCLEARED    = 7,
  STATUS_JOY          = 8,
  STATUS_ESTOPPED     = 9,
  NUM_STATUS          = 10
};

enum Commands
{
  NEW_MAP       = 0,
  GENERATE_PLAN = 1,
  START_SWEEP   = 2,
  NUM_COMMANDS  = 3
};

enum MouseState
{
  PAN              = 0,
  GEN_PLAN         = 1,
  ADD_POSE         = 2,
  REM_POSES        = 3,
  NUM_MOUSE_STATES = 4
};

enum EditState
{
  EPAN            = 0,
  ERASE           = 1,
  DRAW            = 2,
  LINE            = 3,
  TEXT            = 4,
  NUM_EDIT_STATES = 5
};

} // namespace sweep_gui

#endif // _SWEEP_GUI_ENUMS_H_
