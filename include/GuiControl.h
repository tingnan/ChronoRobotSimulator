#ifndef GUICONTROL_H
#define GUICONTROL_H
#include <irrlicht.h>
#include "include/Robot.h"

struct GlobalControlSet {
  double gTimeStep;
  SnakeControlSet snakeParams;
};

namespace irr {
//********************
// EVENT RECEIVER CLASS
//********************
class MyEventReceiver : public IEventReceiver {
private:
  bool KeyIsDown;
  class ChIrrApp *mApp;
  gui::IGUICheckBox *mChBoxPauseSim;
  gui::IGUIEditBox *mEdBoxTimeStep;
  gui::IGUIEditBox *mEdBox_k;
  gui::IGUIEditBox *mEdBox_A;
  gui::IGUIEditBox *mEdBox_w;
  gui::IGUIEditBox *mEdBox_h;
  GlobalControlSet *mControlSet;

public:
  MyEventReceiver(class ChIrrApp *, GlobalControlSet *);
  bool OnEvent(const SEvent &event);
  // update text when global params are changed
  void UpdateText();
};
}
#endif
