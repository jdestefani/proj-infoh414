#include "landmarks.h"
#include <argos3/core/wrappers/lua/lua_controller.h>

/****************************************/
/****************************************/

static const size_t TARGETS        = 5;
static const Real   TARGET_RADIUS  = 0.3;
static const Real   TARGET_RADIUS2 = TARGET_RADIUS * TARGET_RADIUS;

/****************************************/
/****************************************/

CLandmarks::CLandmarks() :
   m_vecAtTarget(TARGETS, false),
   m_vecTargets(TARGETS) {
   m_vecTargets[0].Set(-3.5, -4.5);
   m_vecTargets[1].Set(-3.5, 4);
   m_vecTargets[2].Set(1, 4.5);
   m_vecTargets[3].Set(3.5, 3);
   m_vecTargets[4].Set(3, -4.5);
}

/****************************************/
/****************************************/

CLandmarks::~CLandmarks() {
}

/****************************************/
/****************************************/

void CLandmarks::Init(TConfigurationNode& t_tree) {
   GetNodeAttribute(t_tree, "output", m_strOutFile);
   m_cOutFile.open(m_strOutFile.c_str(), std::ofstream::out | std::ofstream::trunc);
   if(m_cOutFile.fail()) {
      THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutFile << "\"");
   }
}

/****************************************/
/****************************************/

void CLandmarks::Reset() {
   m_cOutFile.close();
   if(m_cOutFile.fail()) {
      THROW_ARGOSEXCEPTION("Error closing file \"" << m_strOutFile << "\"");
   }
   m_cOutFile.open(m_strOutFile.c_str(), std::ofstream::out | std::ofstream::trunc);
   if(m_cOutFile.fail()) {
      THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutFile << "\"");
   }
}

/****************************************/
/****************************************/

void CLandmarks::Destroy() {
}

/****************************************/
/****************************************/

void CLandmarks::PreStep() {
}

/****************************************/
/****************************************/

void CLandmarks::PostStep() {
   m_vecAtTarget = std::vector<bool>(TARGETS, false);
   SInt32 nAtTarget;
   size_t unInChain = 0;
   CSpace::TMapPerType& cFBMap = m_cSpace.GetEntitiesByType("footbot");
   for(CSpace::TMapPerType::iterator it = cFBMap.begin(); it != cFBMap.end(); ++it) {
      CFootBotEntity& cFB = *any_cast<CFootBotEntity*>(it->second);
      nAtTarget = AtTarget(cFB);
      if(nAtTarget >= 0) m_vecAtTarget[nAtTarget] = true;
      if(InChain(cFB)) ++unInChain;
   }
   m_cOutFile << m_cSpace.GetSimulationClock() << "\t"
              << unInChain << std::endl;
}

/****************************************/
/****************************************/

bool CLandmarks::IsExperimentFinished() {
   for(size_t i = 0; i < m_vecAtTarget.size(); ++i) {
      if(!m_vecAtTarget[i]) {
         return false;
      }
   }
   return true;
}

/****************************************/
/****************************************/

CColor CLandmarks::GetFloorColor(const CVector2& c_position_on_plane) {
   if((c_position_on_plane.GetX() >= -2.0f &&
       c_position_on_plane.GetX() <= 2.0f) &&
      (c_position_on_plane.GetY() >= -5.0f &&
       c_position_on_plane.GetY() <= -1.0f)) {
      return CColor::GRAY90;
   }
   for(size_t i = 0; i < TARGETS; ++i) {
      if(SquareDistance(c_position_on_plane, m_vecTargets[i]) < TARGET_RADIUS2) {
         return CColor::BLACK;
      }
   }
   return CColor::WHITE;
}

/****************************************/
/****************************************/

bool CLandmarks::InChain(CFootBotEntity& c_fb) {
   bool bRetVal = false;
   CLuaController& cContr = dynamic_cast<CLuaController&>(c_fb.GetControllableEntity().GetController());
   lua_getglobal(cContr.GetLuaState(), "robot");
   lua_getfield(cContr.GetLuaState(), -1, "in_chain");
   if((! lua_isnil(cContr.GetLuaState(), -1)) &&
      (lua_isnumber(cContr.GetLuaState(), -1))) {
      Real fInChain = lua_tonumber(cContr.GetLuaState(), -1);
      if(fInChain != 0.0) {
         bRetVal = true;
      }
   }
   lua_pop(cContr.GetLuaState(), 2);
   return bRetVal;
}

/****************************************/
/****************************************/

SInt32 CLandmarks::AtTarget(CFootBotEntity& c_fb) {
   for(size_t i = 0; i < TARGETS; ++i) {
      if(SquareDistance(CVector2(c_fb.GetEmbodiedEntity().GetPosition().GetX(),
                                 c_fb.GetEmbodiedEntity().GetPosition().GetY()),
                        m_vecTargets[i]) < TARGET_RADIUS2) {
         return i;
      }
   }
   return -1;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CLandmarks, "landmarks");
