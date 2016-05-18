#include <chrono_irrlicht/ChIrrApp.h>

#include "include/controller.h"
#include "include/gui.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

inline int wtoi(const wchar_t *str) { return (int)wcstol(str, 0, 10); }

MyEventReceiver::MyEventReceiver(chrono::irrlicht::ChIrrApp *app,
                                 Controller *controller)
    : ch_app_(app), controller_(controller), space_key_down_(false) {
  gui::IGUIEnvironment *mygui = ch_app_->GetIGUIEnvironment();

  // now let us define some gui layout;
  mygui
      ->addStaticText(L"pause physics:", core::rect<s32>(220, 10, 300, 30),
                      false, false, 0, 101)
      ->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  checkbox_pause_sim_ =
      mygui->addCheckBox(false, core::rect<s32>(300, 10, 320, 30), 0, 102, L"");
  ch_app_->SetPaused(checkbox_pause_sim_->isChecked());

  mygui
      ->addStaticText(L"time step:", core::rect<s32>(330, 10, 380, 30), false,
                      false, 0, 103)
      ->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  edbox_time_step_ = mygui->addEditBox(
      L"1e-2", core::rect<s32>(380, 10, 430, 30), true, 0, 104);
  edbox_time_step_->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  double time_step = wcstod(edbox_time_step_->getText(), NULL);
  ch_app_->SetTimestep(time_step);
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
        ch_app_->SetPaused(checkbox_pause_sim_->isChecked());
      }
      break;
    default:
      break;
    }
  }

  // Key EVENT
  if (event.EventType == EET_KEY_INPUT_EVENT) {
    if (event.KeyInput.Key == KEY_SPACE) {
      if (event.KeyInput.PressedDown == true && space_key_down_ == false) {
        space_key_down_ = true;
        checkbox_pause_sim_->setChecked(!(checkbox_pause_sim_->isChecked()));
      }
      if (event.KeyInput.PressedDown == false && space_key_down_ == true) {
        space_key_down_ = false;
      }
    }

    // Json::Value command;
    // if (event.KeyInput.Key == KEY_UP && event.KeyInput.PressedDown) {
    //
    //   command["amplitude"] = 1.00;
    //   command["count_down"] = 250;
    //   controller_->PushCommandToQueue(command);
    //   // controller_->SetDefaultAmplitude(0.80);
    // }
    // if (event.KeyInput.Key == KEY_LEFT && event.KeyInput.PressedDown) {
    //   command["amplitude"] = 1.00;
    //   command["count_down"] = 125;
    //   controller_->PushCommandToQueue(command);
    //
    //   // controller_->SetDefaultAmplitude(0.10);
    // }
    // if (event.KeyInput.Key == KEY_RIGHT && event.KeyInput.PressedDown) {
    //   command["amplitude"] = 0.80;
    //   command["count_down"] = 375;
    //   controller_->PushCommandToQueue(command);
    //
    //   // controller_->SetDefaultAmplitude(0.10);
    // }
  }

  return false;
}
