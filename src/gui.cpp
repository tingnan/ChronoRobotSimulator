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
  mApp = myapp;
  mControlSet = myset;
  gui::IGUIEnvironment *mygui = myapp->GetIGUIEnvironment();

  // now let us define some gui layout;
  mygui->addStaticText(L"pause physics:", core::rect<s32>(220, 10, 300, 30),
                       false, false, 0,
                       101)->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  mChBoxPauseSim =
      mygui->addCheckBox(false, core::rect<s32>(300, 10, 320, 30), 0, 102, L"");
  mApp->SetPaused(mChBoxPauseSim->isChecked());

  mygui->addStaticText(L"time step:", core::rect<s32>(330, 10, 380, 30), false,
                       false, 0,
                       103)->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  mEdBoxTimeStep = mygui->addEditBox(L"1e-3", core::rect<s32>(380, 10, 430, 30),
                                     true, 0, 104);
  mEdBoxTimeStep->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  mControlSet->gTimeStep = wcstod(mEdBoxTimeStep->getText(), NULL);

  mygui->addStaticText(L"k:", core::rect<s32>(440, 10, 470, 30), false, false,
                       0, 105)->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  mEdBox_k =
      mygui->addEditBox(L"25", core::rect<s32>(470, 10, 500, 30), true, 0, 106);
  mEdBox_k->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  mControlSet->snakeParams.k = wcstod(mEdBox_k->getText(), NULL);

  mygui->addStaticText(L"A:", core::rect<s32>(510, 10, 540, 30), false, false,
                       0, 107)->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  mEdBox_A = mygui->addEditBox(L"0.50", core::rect<s32>(540, 10, 570, 30), true,
                               0, 108);
  mEdBox_A->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  mControlSet->snakeParams.A = wcstod(mEdBox_A->getText(), NULL);

  mygui->addStaticText(L"freq:", core::rect<s32>(580, 10, 620, 30), false,
                       false, 0,
                       109)->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  mEdBox_w = mygui->addEditBox(L"0.2", core::rect<s32>(620, 10, 650, 30), true,
                               0, 110);
  mEdBox_w->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  mControlSet->snakeParams.w = wcstod(mEdBox_w->getText(), NULL);

  mygui->addStaticText(L"height:", core::rect<s32>(660, 10, 700, 30), false,
                       false, 0,
                       111)->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  mEdBox_h = mygui->addEditBox(L"-0.26", core::rect<s32>(700, 10, 740, 30),
                               true, 0, 112);
  mEdBox_h->setTextAlignment(EGUIA_CENTER, EGUIA_CENTER);
  mControlSet->snakeParams.h = wcstod(mEdBox_h->getText(), NULL);

  KeyIsDown = false;
}

void MyEventReceiver::UpdateText() {
  wchar_t valuestring[256];
  swprintf(valuestring, 256, L"%1.1e", mControlSet->gTimeStep);
  mEdBoxTimeStep->setText(valuestring);
  // set k
  swprintf(valuestring, 256, L"%1.0f", mControlSet->snakeParams.k);
  mEdBox_k->setText(valuestring);

  swprintf(valuestring, 256, L"%1.2f", mControlSet->snakeParams.A);
  mEdBox_A->setText(valuestring);

  swprintf(valuestring, 256, L"%1.2f", mControlSet->snakeParams.w);
  mEdBox_w->setText(valuestring);

  swprintf(valuestring, 256, L"%1.3f", mControlSet->snakeParams.h);
  mEdBox_h->setText(valuestring);
}

bool MyEventReceiver::OnEvent(const SEvent &event) {
  // GUI EVENT
  if (event.EventType == EET_GUI_EVENT) {
    s32 id = event.GUIEvent.Caller->getID();

    switch (event.GUIEvent.EventType) {
    case gui::EGET_CHECKBOX_CHANGED:
      if (id == 102) {
        mApp->SetPaused(mChBoxPauseSim->isChecked());
      }
      break;
    case gui::EGET_EDITBOX_ENTER:
      if (id == 104) {
        mControlSet->gTimeStep = wcstod(mEdBoxTimeStep->getText(), NULL);
        break;
      }
      if (id == 106) {
        mControlSet->snakeParams.k = wtoi(mEdBox_k->getText());
        break;
      }
      if (id == 108) {
        mControlSet->snakeParams.A = wcstod(mEdBox_A->getText(), NULL);
        break;
      }
      if (id == 110) {
        mControlSet->snakeParams.w = wcstod(mEdBox_w->getText(), NULL);
        break;
      }
      if (id == 112) {
        mControlSet->snakeParams.h = wcstod(mEdBox_h->getText(), NULL);
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
      if (event.KeyInput.PressedDown == true && KeyIsDown == false) {
        KeyIsDown = true;
        mChBoxPauseSim->setChecked(!(mChBoxPauseSim->isChecked()));
      }

      if (event.KeyInput.PressedDown == false && KeyIsDown == true) {
        KeyIsDown = false;
      }
    }
    if (event.KeyInput.Key == KEY_ESCAPE) {
    }
  }

  return false;
}
