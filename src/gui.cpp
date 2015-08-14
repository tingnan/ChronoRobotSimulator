#include <unit_IRRLICHT/ChIrrApp.h>

#include "include/gui.h"
#include "include/robot.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

inline int wtoi(const wchar_t *str) { return (int)wcstol(str, 0, 10); }

MyEventReceiver::MyEventReceiver(ChIrrApp *myapp) {
  app_ = myapp;
  gui::IGUIEnvironment *mygui = myapp->GetIGUIEnvironment();

  // now let us define some gui layout;
  mygui->addStaticText(L"pause physics:", core::rect<s32>(220, 10, 300, 30),
                       false, false, 0,
                       101)->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  checkbox_pause_sim_ =
      mygui->addCheckBox(false, core::rect<s32>(300, 10, 320, 30), 0, 102, L"");
  app_->SetPaused(checkbox_pause_sim_->isChecked());

  mygui->addStaticText(L"time step:", core::rect<s32>(330, 10, 380, 30), false,
                       false, 0,
                       103)->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  edbox_time_step_ = mygui->addEditBox(
      L"1e-3", core::rect<s32>(380, 10, 430, 30), true, 0, 104);
  edbox_time_step_->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  double time_step = wcstod(edbox_time_step_->getText(), NULL);
  myapp->SetTimestep(time_step);
  std::cout << time_step << std::endl;
  key_down_ = false;
}

void MyEventReceiver::UpdateText() {
  // wchar_t valuestring[256];
  // swprintf(valuestring, 256, L"%1.1e", time_step);
  // edbox_time_step_->setText(valuestring);
}

bool MyEventReceiver::OnEvent(const SEvent &event) {
  // GUI EVENT
  if (event.EventType == EET_GUI_EVENT) {
    s32 id = event.GUIEvent.Caller->getID();

    switch (event.GUIEvent.EventType) {
    case gui::EGET_CHECKBOX_CHANGED:
      if (id == 102) {
        app_->SetPaused(checkbox_pause_sim_->isChecked());
      }
      break;
    default:
      break;
    }
  }

  // Key EVENT
  if (event.EventType == EET_KEY_INPUT_EVENT) {
    if (event.KeyInput.Key == KEY_SPACE) {
      if (event.KeyInput.PressedDown == true && key_down_ == false) {
        key_down_ = true;
        checkbox_pause_sim_->setChecked(!(checkbox_pause_sim_->isChecked()));
      }

      if (event.KeyInput.PressedDown == false && key_down_ == true) {
        key_down_ = false;
      }
    }
  }

  return false;
}
