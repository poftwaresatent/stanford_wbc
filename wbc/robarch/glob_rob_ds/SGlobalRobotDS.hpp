#ifndef SGLOBALROBOTDS_HPP_
#define SGLOBALROBOTDS_HPP_

namespace wbc_tinyxml {
class TiXmlElement;
}

namespace robotarchitect
{

typedef float wbcFloat;

//NOTE TODO we could consider removing these enums:
typedef enum{
  Null_Joint_Tag, Shoulder_Yaw, Shoulder_Pitch,
  Shoulder_Roll, Elbow, Wrist_Roll1,
  Wrist_Pitch, Wrist_Roll2 } JointTag;

typedef enum{
  Null_Link_Tag, Upper_Arm, Lower_Arm, End_Effector,
	Right_Hand, Left_Hand , Hip, Right_Foot, Left_Foot} LinkTag;

typedef enum {
	JT_PRISMATIC = 0, JT_REVOLUTE = 1, JT_SPHERICAL = 2,
	JT_MAX = 3, JT_NOTASSIGNED = -1}JointType;

/**This structure contains all the non-robot specification
 * information required to construct a robotic world.
 * Individual robot definitions are required in addition to this
 * in order to create a robotic simulation environment with controllable
 * robots.
 */
struct SGlobalRobotDS
{
public:
	//Constructor@End of the class:

	//***********************
  //Link-specific data:
	wbcFloat gravity_[3];
};
}


#endif //SGLOBALROBOTDS_HPP_
