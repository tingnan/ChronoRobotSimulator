#ifndef INCLUDE_GUI_H_
#define INCLUDE_GUI_H_

#include <irrlicht.h>

#include "include/robot.h"
namespace irr {
//********************
// EVENT RECEIVER CLASS
//********************
class MyEventReceiver : public IEventReceiver {
public:
  MyEventReceiver(class ChIrrApp *);
  bool OnEvent(const SEvent &event);
  // update text when global params are changed
  void UpdateText();

private:
  bool key_down_;
  class ChIrrApp *app_;
  gui::IGUICheckBox *checkbox_pause_sim_;
  gui::IGUIEditBox *edbox_time_step_;
};
} // namespace irr

#endif // INCLUDE_GUI_H_
