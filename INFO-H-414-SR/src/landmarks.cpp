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
   m_vecAtTarget(TARGETS, false),   // no robot is on a target
   m_vecTargets(TARGETS) {          // empty target locations
   /* Initialize the target locations */
   m_vecTargets[0].Set(-3.5, -4.5);
   m_vecTargets[1].Set(-3.5, 4);
   m_vecTargets[2].Set(1, 4.5);
   m_vecTargets[3].Set(3.5, 3);
   m_vecTargets[4].Set(3, -4.5);
}

/****************************************/
/****************************************/

CLandmarks::~CLandmarks() {
   /* Nothing to do */
}

/****************************************/
/****************************************/

void CLandmarks::Init(TConfigurationNode& t_tree) {
   /* Get output file name from XML tree */
   GetNodeAttribute(t_tree, "output", m_strOutFile);
   /* Open the file for text writing */
   m_cOutFile.open(m_strOutFile.c_str(), std::ofstream::out | std::ofstream::trunc);
   if(m_cOutFile.fail()) {
      THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutFile << "\"");
   }
   /* Write a header line */
   m_cOutFile << "# Clock\tIn Chain\tOn Target" << std::endl;
}

/****************************************/
/****************************************/

void CLandmarks::Reset() {
   /* Set to false the target flags */
   m_vecAtTarget = std::vector<bool>(TARGETS, false);
   /* Close the output file */
   m_cOutFile.close();
   if(m_cOutFile.fail()) {
      THROW_ARGOSEXCEPTION("Error closing file \"" << m_strOutFile << "\"");
   }
   /* Open the file for text writing */
   m_cOutFile.open(m_strOutFile.c_str(), std::ofstream::out | std::ofstream::trunc);
   if(m_cOutFile.fail()) {
      THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutFile << "\"");
   }
   /* Write a header line */
   m_cOutFile << "# Clock\tIn Chain\tOn Target" << std::endl;
}

/****************************************/
/****************************************/

void CLandmarks::Destroy() {
   /* Close the output file */
   m_cOutFile.close();
   if(m_cOutFile.fail()) {
      THROW_ARGOSEXCEPTION("Error closing file \"" << m_strOutFile << "\"");
   }
}

/****************************************/
/****************************************/

void CLandmarks::PreStep() {
   /* Nothing to do */
}

/****************************************/
/****************************************/

void CLandmarks::PostStep() {
   /* Reset flags for robot at target to false */
   m_vecAtTarget = std::vector<bool>(TARGETS, false);
   /* Index for the target on which a foot-bot is located */
   SInt32 nAtTarget;
   /* Counter for the robots currently on a target */
   size_t unOnTarget = 0;
   /* Counter for the robots currently in a chain */
   size_t unInChain = 0;
   /*
    * Get a map of all the foot-bots
    * The map is organized by robot id
    */
   CSpace::TMapPerType& cFBMap = m_cSpace.GetEntitiesByType("footbot");
   /* Go through each foot-bot */
   for(CSpace::TMapPerType::iterator it = cFBMap.begin(); it != cFBMap.end(); ++it) {
      /* Create a reference for easier coding */
      CFootBotEntity& cFB = *any_cast<CFootBotEntity*>(it->second);
      /* Check whether this foot-bot is on a target */
      nAtTarget = AtTarget(cFB);
      if(nAtTarget >= 0) {
         /* Yes, set corresponding flag to true */
         m_vecAtTarget[nAtTarget] = true;
         /* Increase counter */
         ++unOnTarget;
      }
      /* Check whether foot-bot is part of a chain */
      if(InChain(cFB)) {
         /* Yes, increase counter */
         ++unInChain;
      }
   }
   /* Output a line for this step */
   m_cOutFile << m_cSpace.GetSimulationClock() << "\t"
              << unInChain << "\t"
              << unOnTarget
              << std::endl;
}

/****************************************/
/****************************************/

bool CLandmarks::IsExperimentFinished() {
   /*
    * Go through all target locations
    * If at least one target location is not occupied, return false
    * Otherwise return true
    */
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
   /* Return value: true = robot is part of a chain, false otherwise */
   bool bRetVal = false;
   /* Get reference to robot controller */
   CLuaController& cContr = dynamic_cast<CLuaController&>(c_fb.GetControllableEntity().GetController());
   /* Get 'robot' table */
   lua_getglobal(cContr.GetLuaState(), "robot");
   /* Get 'in_chain' variable */
   lua_getfield(cContr.GetLuaState(), -1, "in_chain");
   if((! lua_isnil(cContr.GetLuaState(), -1)) &&              // 'in_chain' variable found
      (lua_isnumber(cContr.GetLuaState(), -1))) {             // its value is a number
      /* Get value as a real number */
      Real fInChain = lua_tonumber(cContr.GetLuaState(), -1);
      /* If not zero, robot is in a chain */
      if(fInChain != 0.0) {
         bRetVal = true;
      }
   }
   /* Clean up Lua stack */
   lua_pop(cContr.GetLuaState(), 2);
   /* Return result */
   return bRetVal;
}

/****************************************/
/****************************************/

SInt32 CLandmarks::AtTarget(CFootBotEntity& c_fb) {
   /*
    * Go through the targets
    * If the square distance between the foot-bot center and the target center
    * is less than the target radius squared, return the id of the current target
    * Otherwise, return -1
    */
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

/* Register this loop functions into the ARGoS plugin system */
REGISTER_LOOP_FUNCTIONS(CLandmarks, "landmarks");
