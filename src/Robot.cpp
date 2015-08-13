#include "unit_IRRLICHT/ChIrrApp.h"
#include "assets/ChColorAsset.h"
#include "physics/ChBodyEasy.h"

#include "include/ChFunction_SquareWave.h"
#include "include/Controller.h"
#include "include/Robot.h"
#include "include/RFT.h"

using namespace chrono;
using irr::ChIrrApp;

ChronoRobotBuilder::ChronoRobotBuilder(ChIrrApp *pApp)
    : mApp(pApp), mSnakeParams(NULL), mRFT(NULL) {}

double SegA(double s, double A) {
  // relative frame rotation between each seg;
  return A * cos(CH_C_2PI * s);
}

/*
void
ChronoRobotBuilder::BuildRobot()
{
    ChSystem *mChSys = mApp->GetSystem();


    if (true)

    {
        // the relation between curvature and the
        // thm * 2pi = AA * kk / delta_k
        if (!mSnakeParams)
        {
            std::cout << "no control params specified!\n" << std::endl;
            return;
        }

        double AA = mSnakeParams->A;
        double kk = mSnakeParams->k;
        double ww = mSnakeParams->w;
        double hh = mSnakeParams->h;

        std::cout << kk << "\t" << AA << "\t" << ww << std::endl;
        const int kNseg = 12 + 11;
        mController = new RobotController();
        mController->SetControlSet(mSnakeParams);


        const double friction = 0.2;
        std::vector<ChSharedBodyPtr> body_container_;
        ChSharedPtr<ChBodyEasyBox> ground(new ChBodyEasyBox(20, 1.0, 20, 1e3,
false, false));
        ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset);
        mcolor->SetColor(ChColor(0.1f, 0.1f, 0.8f));
        ground->SetPos(ChVector<>(0, -0.5, 0.));
        ground->AddAsset(mcolor);
        ground->SetBodyFixed(true);
        ground->SetIdentifier(-1);
        ground->SetFriction(friction);
        mChSys->AddBody(ground);


        const double lx = 0.10;
        const double ly = 0.05;
        const double lz = 0.05;
        const double rho = 1450;
        const double ypos = hh;
        const double zpos = 0.25;
        double lastx = 0;

        // construct the body
        // the initial frame aligned with world frame
        ChFrame<> lastframe(ChVector<>(lastx, ypos, zpos));
        for (int k = 0; k < kNseg; ++k)
        {
            // translate and rotate
            if (k % 2 == 0)
            {
                ChSharedPtr<ChBodyEasyBox> chShape(new ChBodyEasyBox(lx, ly, lz,
rho, true, true));
                body_container_.push_back(chShape);
                chShape->SetIdentifier(k);
                chShape->SetFriction(friction);
                mChSys->AddBody(chShape);
                mRFTBodylist.push_back(RFTBody(chShape.get_ptr()));

                chShape->SetPos(lastframe * ChVector<>(0.5 * lx, 0, 0));
                chShape->SetRot(lastframe.GetRot());
                ChVector<> enginePos(lastframe * ChVector<>(0, 0, 0));

                if (k > 1)
                {
                    // add the engine
                    ChSharedPtr<ChLinkEngine> mylink(new ChLinkEngine);
                    //mylink->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_PRISM);
                    mylink->SetIdentifier(k + 100);
                    ChQuaternion<> engineOri(Q_from_AngX(CH_C_PI_2));
                    mylink->Initialize(body_container_[k], body_container_[k -
2], ChCoordsys<>(enginePos, engineOri));
                    mChSys->AddLink(mylink);
                    mController->AddEngine(mylink.get());
                }

                    ChSharedPtr<ChLinkLockPlanePlane> inplanelink(new
ChLinkLockPlanePlane);
                    inplanelink->SetIdentifier(1000 + k);
                    inplanelink->Initialize(ground, body_container_[k],
ChCoordsys<>(ChVector<>(), Q_from_AngX(CH_C_PI_2)));
                    mChSys->AddLink(inplanelink);

                    lastframe = ChFrame<>(ChVector<>(lx, 0, 0), Q_from_AngY(0 *
SegA(double(k) / kNseg, AA))) >> lastframe;
            }

            if (k % 2 == 1 && k < kNseg - 1)
            {

                // add a cylinder segment to protect the joint
                ChSharedPtr<ChBodyEasySphere> chShape(new ChBodyEasySphere(ly /
2, rho * 0.01, true, true));
                body_container_.push_back(chShape);
                chShape->SetIdentifier(k);
                chShape->SetFriction(friction);
                mChSys->AddBody(chShape);
                ChVector<> enginePos(lastframe * ChVector<>(0, 0, 0));
                chShape->SetPos(enginePos);
                chShape->SetRot(Q_from_AngY(CH_C_PI_2));

                ChSharedPtr<ChLinkLockLock> mylink(new ChLinkLockLock);
                mylink->SetIdentifier(k + 100);
                mylink->Initialize(body_container_[k], body_container_[k - 1],
ChCoordsys<>(enginePos));
                mChSys->AddLink(mylink);

            }
        }

        if (true)
        {
            // now add the head
            ChSharedPtr<ChBodyEasyCylinder> chShape(new ChBodyEasyCylinder(0.5 *
lz, ly, rho * 0.01, true, true));
            body_container_.push_back(chShape);
            chShape->SetIdentifier(kNseg);
            chShape->SetFriction(friction);
            mChSys->AddBody(chShape);
            mRFTBodylist.push_back(RFTBody(chShape.get_ptr()));
            ChVector<> enginePos(lastframe * ChVector<>(0, 0, 0));
            chShape->SetPos(enginePos);
            chShape->SetRot(Q_from_AngY(CH_C_PI_2));

            ChSharedPtr<ChLinkLockLock> mylink(new ChLinkLockLock);
            mylink->SetIdentifier(kNseg + 100);
            mylink->Initialize(body_container_[kNseg], body_container_[kNseg -
1], ChCoordsys<>(enginePos));
            mChSys->AddLink(mylink);

        }

    }
    mController->PositionControl();
    mApp->AssetBindAll();
    mApp->AssetUpdateAll();
}
*/

void ChronoRobotBuilder::BuildRobot() {

  ChSystem *mChSys = mApp->GetSystem();
  if (true) {
    // the relation between curvature and the
    // thm * 2pi = AA * kk / delta_k
    if (!mSnakeParams) {
      std::cout << "no control params specified!\n" << std::endl;
      return;
    }

    double AA = mSnakeParams->A;
    double kk = mSnakeParams->k;
    double ww = mSnakeParams->w;
    double hh = mSnakeParams->h;

    std::cout << kk << "\t" << AA << "\t" << ww << std::endl;
    const int kNseg = 25 + 24;
    mController = new RobotController();
    mController->SetControlSet(mSnakeParams);

    const double friction = 0.1;
    std::vector<ChSharedBodyPtr> body_container_;
    ChSharedPtr<ChBodyEasyBox> ground(
        new ChBodyEasyBox(200, 1.0, 200, 1e3, false, false));
    ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset);
    mcolor->SetColor(ChColor(0.1f, 0.1f, 0.8f));
    ground->SetPos(ChVector<>(0, -0.5, 0.));
    ground->AddAsset(mcolor);
    ground->SetBodyFixed(true);
    ground->SetIdentifier(-1);
    ground->GetMaterialSurface()->SetFriction(friction);

    mChSys->AddBody(ground);

    const double sf = 5;
    const double lx = 0.04 * sf;
    const double ly = 0.02 * sf;
    const double lz = 0.02 * sf;
    const double rho = 1450 / (sf * sf);
    const double ypos = hh;
    double lastx = 0.5;

    // construct the body
    // the initial frame aligned with world frame
    ChFrame<> lastframe(ChVector<>(lastx, ypos, 0));
    for (int k = 0; k < kNseg; ++k) {
      // translate and rotate
      if (k % 2 == 0) {
        ChSharedPtr<ChBodyEasyBox> chShape(
            new ChBodyEasyBox(lx, ly, lz, rho, true, true));
        body_container_.push_back(chShape);
        chShape->SetIdentifier(k);
        chShape->GetMaterialSurface()->SetFriction(friction);
        mChSys->AddBody(chShape);
        mRFTBodylist.push_back(RFTBody(chShape.get_ptr()));

        chShape->SetPos(lastframe * ChVector<>(0.5 * lx, 0, 0));
        chShape->SetRot(lastframe.GetRot());
        ChVector<> enginePos(lastframe * ChVector<>(0, 0, 0));

        if (k > 1) {
          // add the engine
          ChSharedPtr<ChLinkEngine> mylink(new ChLinkEngine);
          // mylink->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_PRISM);
          mylink->SetIdentifier(k + 100);
          ChQuaternion<> engineOri(Q_from_AngX(CH_C_PI_2));
          mylink->Initialize(body_container_[k], body_container_[k - 2],
                             ChCoordsys<>(enginePos, engineOri));
          mChSys->AddLink(mylink);
          mController->AddEngine(mylink.get());
        }

        ChSharedPtr<ChLinkLockPlanePlane> inplanelink(new ChLinkLockPlanePlane);
        inplanelink->SetIdentifier(1000 + k);
        inplanelink->Initialize(
            ground, body_container_[k],
            ChCoordsys<>(ChVector<>(), Q_from_AngX(CH_C_PI_2)));
        mChSys->AddLink(inplanelink);

        lastframe = ChFrame<>(ChVector<>(lx, 0, 0),
                              Q_from_AngY(0 * SegA(double(k) / kNseg, AA))) >>
                    lastframe;
      }

      if (k % 2 == 1 && k < kNseg - 1) {

        // add a cylinder segment to protect the joint
        ChSharedPtr<ChBodyEasySphere> chShape(
            new ChBodyEasySphere(ly / 2, rho * 0.01, true, true));
        body_container_.push_back(chShape);
        chShape->SetIdentifier(k);
        chShape->GetMaterialSurface()->SetFriction(friction);
        mChSys->AddBody(chShape);
        ChVector<> enginePos(lastframe * ChVector<>(0, 0, 0));
        chShape->SetPos(enginePos);
        chShape->SetRot(Q_from_AngY(CH_C_PI_2));

        ChSharedPtr<ChLinkLockLock> mylink(new ChLinkLockLock);
        mylink->SetIdentifier(k + 100);
        mylink->Initialize(body_container_[k], body_container_[k - 1],
                           ChCoordsys<>(enginePos));
        mChSys->AddLink(mylink);
      }
    }

    if (true) {
      // now add the head
      ChSharedPtr<ChBodyEasyCylinder> chShape(
          new ChBodyEasyCylinder(0.5 * lz, ly, rho * 0.01, true, true));
      body_container_.push_back(chShape);
      chShape->SetIdentifier(kNseg);
      chShape->GetMaterialSurface()->SetFriction(friction);
      mChSys->AddBody(chShape);
      // mRFTBodylist.push_back(RFTBody(chShape.get_ptr()));
      ChVector<> enginePos(lastframe * ChVector<>(0, 0, 0));
      chShape->SetPos(enginePos);
      chShape->SetRot(Q_from_AngY(CH_C_PI_2));

      ChSharedPtr<ChLinkLockLock> mylink(new ChLinkLockLock);
      mylink->SetIdentifier(kNseg + 100);
      mylink->Initialize(body_container_[kNseg], body_container_[kNseg - 1],
                         ChCoordsys<>(enginePos));
      mChSys->AddLink(mylink);
    }
  }
  mController->PositionControl();
}

ChVector<> ChronoRobotBuilder::GetRobotCoMPosition() {
  ChVector<> pos;
  const size_t nnode = mRFTBodylist.size();
  double total_mass = 0;
  for (int i = 0; i < nnode; ++i) {
    ChVector<> tmppos = mRFTBodylist[i].GetChBody()->GetPos();
    double tmpmass = mRFTBodylist[i].GetChBody()->GetMass();
    pos += tmppos * tmpmass;
    total_mass += tmpmass;
  }
  return pos / total_mass;
}

void ChronoRobotBuilder::BuildBoard(double spacing) {
  ChSystem *mChSys = mApp->GetSystem();
  const int kNpeg = 30;
  double pegSeparation = spacing;

  for (int i = 0; i < kNpeg; ++i) {
    for (int j = 0; j < kNpeg; ++j) {
      double xPos = i * pegSeparation;
      double zPos = (j - kNpeg / 2) * pegSeparation;
      ChSharedPtr<ChBodyEasyCylinder> chShape(
          new ChBodyEasyCylinder(0.05, 3, 1, true, true));
      chShape->SetIdentifier(-1);
      chShape->GetMaterialSurface()->SetFriction(0.0);
      chShape->SetPos(ChVector<>(xPos, 0, zPos));
      chShape->SetBodyFixed(true);
      mChSys->AddBody(chShape);
      mCollisionObjs.push_back(chShape.get_ptr());
    }
  }

  if (false) {

    for (int i = -1; i <= 1; i += 2) {
      ChSharedPtr<ChBodyEasyBox> chShape(
          new ChBodyEasyBox(100, 3, 0.2, 1, true, true));
      chShape->SetIdentifier(-1);
      chShape->GetMaterialSurface()->SetFriction(0.0);
      chShape->SetBodyFixed(true);
      chShape->SetPos(ChVector<>(50, 0, i * 1.2));
      mChSys->AddBody(chShape);
    }
  }
}

void ChronoRobotBuilder::SetCollide(bool mcol) {
  for (int i = 0; i < mCollisionObjs.size(); ++i) {
    mCollisionObjs[i]->SetCollide(mcol);
  }
}
