#ifndef INCLUDE_GUI_H_
#define INCLUDE_GUI_H_

#include <chrono_irrlicht/ChIrrApp.h>
#include <irrlicht.h>
class Controller;
namespace irr {
//********************
// EVENT RECEIVER CLASS
//********************
class MyEventReceiver : public IEventReceiver {
public:
  MyEventReceiver(chrono::irrlicht::ChIrrApp *ch_app, Controller *controller);
  bool OnEvent(const SEvent &event);
  // update text when global params are changed
  void UpdateText();

private:
  bool space_key_down_;
  chrono::irrlicht::ChIrrApp *ch_app_;
  Controller *controller_;
  gui::IGUICheckBox *checkbox_pause_sim_;
  gui::IGUIEditBox *edbox_time_step_;
};
} // namespace irr

#endif // INCLUDE_GUI_H_
