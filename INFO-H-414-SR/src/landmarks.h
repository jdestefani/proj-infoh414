#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <fstream>

using namespace argos;

class CLandmarks : public CLoopFunctions {

public:

   CLandmarks();
   virtual ~CLandmarks();

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();

   virtual void PreStep();
   virtual void PostStep();

   virtual bool IsExperimentFinished();

   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

private:

   bool InChain(CFootBotEntity& c_fb);
   SInt32 AtTarget(CFootBotEntity& c_fb);

private:

   std::vector<bool>     m_vecAtTarget;
   std::vector<CVector2> m_vecTargets;
   std::string           m_strOutFile;
   std::ofstream         m_cOutFile;

};
