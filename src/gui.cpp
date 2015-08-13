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

MyEventReceiver::MyEventReceiver(ChIrrApp *myapp, GlobalControlSet *myset) {
  app_ = myapp;
  control_set_ = myset;
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
  control_set_->gTimeStep = wcstod(edbox_time_step_->getText(), NULL);

  mygui->addStaticText(L"k:", core::rect<s32>(440, 10, 470, 30), false, false,
                       0, 105)->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  ed_box_k_ =
      mygui->addEditBox(L"25", core::rect<s32>(470, 10, 500, 30), true, 0, 106);
  ed_box_k_->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  control_set_->snakeParams.k = wcstod(ed_box_k_->getText(), NULL);

  mygui->addStaticText(L"A:", core::rect<s32>(510, 10, 540, 30), false, false,
                       0, 107)->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  ed_box_a_ = mygui->addEditBox(L"0.50", core::rect<s32>(540, 10, 570, 30),
                                true, 0, 108);
  ed_box_a_->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  control_set_->snakeParams.A = wcstod(ed_box_a_->getText(), NULL);

  mygui->addStaticText(L"freq:", core::rect<s32>(580, 10, 620, 30), false,
                       false, 0,
                       109)->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  ed_box_w_ = mygui->addEditBox(L"0.2", core::rect<s32>(620, 10, 650, 30), true,
                                0, 110);
  ed_box_w_->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  control_set_->snakeParams.w = wcstod(ed_box_w_->getText(), NULL);

  mygui->addStaticText(L"height:", core::rect<s32>(660, 10, 700, 30), false,
                       false, 0,
                       111)->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  ed_box_h_ = mygui->addEditBox(L"-0.26", core::rect<s32>(700, 10, 740, 30),
                                true, 0, 112);
  ed_box_h_->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  control_set_->snakeParams.h = wcstod(ed_box_h_->getText(), NULL);

  key_down_ = false;
}

void MyEventReceiver::UpdateText() {
  wchar_t valuestring[256];
  swprintf(valuestring, 256, L"%1.1e", control_set_->gTimeStep);
  edbox_time_step_->setText(valuestring);
  // set k
  swprintf(valuestring, 256, L"%1.0f", control_set_->snakeParams.k);
  ed_box_k_->setText(valuestring);

  swprintf(valuestring, 256, L"%1.2f", control_set_->snakeParams.A);
  ed_box_a_->setText(valuestring);

  swprintf(valuestring, 256, L"%1.2f", control_set_->snakeParams.w);
  ed_box_w_->setText(valuestring);

  swprintf(valuestring, 256, L"%1.3f", control_set_->snakeParams.h);
  ed_box_h_->setText(valuestring);
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
    case gui::EGET_EDITBOX_ENTER:
      if (id == 104) {
        control_set_->gTimeStep = wcstod(edbox_time_step_->getText(), NULL);
        break;
      }
      if (id == 106) {
        control_set_->snakeParams.k = wtoi(ed_box_k_->getText());
        break;
      }
      if (id == 108) {
        control_set_->snakeParams.A = wcstod(ed_box_a_->getText(), NULL);
        break;
      }
      if (id == 110) {
        control_set_->snakeParams.w = wcstod(ed_box_w_->getText(), NULL);
        break;
      }
      if (id == 112) {
        control_set_->snakeParams.h = wcstod(ed_box_h_->getText(), NULL);
        break;
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
    if (event.KeyInput.Key == KEY_ESCAPE) {
    }
  }

  return false;
}
