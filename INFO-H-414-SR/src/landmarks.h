#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <fstream>

using namespace argos;

class CLandmarks : public CLoopFunctions {

public:

   /**
    * Class constructor
    */
   CLandmarks();

   /** 
    * Class destructor
    */
   virtual ~CLandmarks();

   /**
    * Initializes the experiment.
    * It is executed once at the beginning of the experiment, i.e., when ARGoS is launched.
    * @param t_tree The parsed XML tree corresponding to the <loop_functions> section.
    */
   virtual void Init(TConfigurationNode& t_tree);

   /**
    * Resets the experiment to the state it was right after Init() was called.
    * It is executed every time you press the 'reset' button in the GUI.
    */
   virtual void Reset();

   /**
    * Undoes whatever Init() did.
    * It is executed once when ARGoS has finished the experiment.
    */
   virtual void Destroy();

   /**
    * Performs actions right before a simulation step is executed.
    */
   virtual void PreStep();

   /**
    * Performs actions right after a simulation step is executed.
    */
   virtual void PostStep();

   /**
    * Returns true if the experiment is finished. False otherwise.
    */
   virtual bool IsExperimentFinished();

   /**
    * Returns the color of the floor at the specified point on.
    * @param c_position_on_plane The position at which you want to get the color.
    * @see CColor
    */
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

private:

   /**
    * Returns true if the wanted foot-bot is part of a chain.
    * @param c_fb The foot-bot.
    */
   bool InChain(CFootBotEntity& c_fb);

   /**
    * Returns -1 if the given foot-bot is not on a target; otherwise it returns the index of the target on which the foot-bot is located.
    * @param c_fb The foot-bot.
    */
   SInt32 AtTarget(CFootBotEntity& c_fb);

private:

   /**
    * A vector of flags to know whether a robot is on a specific target
    * (true = at last one robot on that target, false = no robot on that target)
    */
   std::vector<bool>     m_vecAtTarget;

   /**
    * The position of the targets on the ground.
    */
   std::vector<CVector2> m_vecTargets;

   /**
    * The path of the output file.
    */
   std::string           m_strOutFile;

   /**
    * The stream associated to the output file.
    */
   std::ofstream         m_cOutFile;

};
