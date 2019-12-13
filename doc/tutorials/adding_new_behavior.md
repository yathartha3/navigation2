## How to add a new action behavior to navigation2
In this tutorial, we will go throught the step-by-step process of implementing a new behavior for navigation2 using the behavior tree.

[Explain visually and textually how things fit in from an eagle's view.
This includes the BT library, BT ros interface, BT plugin, our new node, bt xml file, etc. Also a sentence about why this makes sense?]
[Add some info and links about behavior tree]

Let's say we want to design a new behavior that moves the robot forward and backwards for a short specified duration once after reacing the goal. we will call this behavior "StepFrontBack". For this hypothetical situation, we will go through all the changes that one would need to make in navigation2 to add this new behavior.

[Explain how these action nodes are plugins]

[Add some visual map of what we are trying to achieve]

#### Step 1 of 4: Define the ros2 action for the desired behavior
[TODO:Need to tie in to why we need the action message]
In ```navigation2/nav2_msgs```, make a new file named ```StepFrontBack.action```, and place the following contents inside it.

```
#goal definition
float32 distance
---
#result definition
std_msgs/Empty result
---
#feedback
```

Here, we are defining the action message, that has the ```distance``` that you want to move front and back.
[TODO:Explain why the result is empty]
[TODO:Explain what you can do with the feedback]

Now, in the ```nav2_msgs/CMakeLists.txt```, add "action/StepFrontBack.action".

_Example:_
```
rosidl_generate_interfaces(${PROJECT_NAME}
  ...
  "action/StepFrontBack.action"
  ...
)
```
So what we have done till now is, created a new action message type, and added it to the CMakeList so that it can be built with nav2_msgs package.


#### Step 2 of 4: Define a behavior tree ros2 node
[TODO: Tie it into why we need to do this]
Now we need to define our node for the action that we want to introduce in the behavior tree. Let's call this behavior tree action node class ```StepFrontBackAction```. Create a new file in ```nav2_behavior_tree/include/```, and name it ```step_forward_back_action.hpp```

Copy and paste the code below in the newly create file.

```
#ifndef NAV2_BEHAVIOR_TREE__STEP_FRONT_BACK_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__STEP_FRONT_BACK_ACTION_HPP_

#include <string>
#include <memory>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/step_front_back.hpp"

namespace nav2_behavior_tree
{

class StepFrontBackAction : public BtActionNode<nav2_msgs::action::StepFrontBack>
{ // this is our BT action node
public:
StepFrontBackAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::StepFrontBack>(action_name, conf)
{
    double dist;
    getInput("distance", dist); //Get the parameter. This will be defined in the xml file
    goal_.distance = dist;
}

static BT::PortsList providedPorts() // define this to get things from the xml file
{
    return providedBasicPorts({
        BT::InputPort<double>("distance", 0.5, "Step front and back distance") // param_name, default, description
    });
}
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__STEP_FRONT_BACK_ACTION_HPP_

```

_Now let's step throught this line-by-line to make it easier to understand what we are trying to achieve with this behavior tree action node:_

```
    #ifndef NAV2_BEHAVIOR_TREE__STEP_FRONT_BACK_ACTION_HPP_
    #define NAV2_BEHAVIOR_TREE__STEP_FRONT_BACK_ACTION_HPP_

    #include <string>
    #include <memory>

    #include "nav2_behavior_tree/bt_action_node.hpp"
    #include "nav2_msgs/action/step_front_back.hpp"

    namespace nav2_behavior_tree
```
Here, we include the header guards, header files and define the namespace.
The header file ```"nav2_msgs/action/step_front_back.hpp"``` is generated after we build the navigation stack after including the ```StepFrontBack.action``` using the process defined in step 1.


```
    class StepFrontBackAction : public BtActionNode<nav2_msgs::action::StepFrontBack>
    {
    public:
    StepFrontBackAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf)
    : BtActionNode<nav2_msgs::action::StepFrontBack>(action_name, conf)
```

Here, we define our behavior tree ros action node.

All navigation2 behavior tree action nodes must derive from the _BtActionNode_ class in ```"nav2_behavior_tree/bt_action_node.hpp"```.

We declare the _action_name_, and the _conf_. The action name can be whatever you want to name the action.
TODO: what is the conf?

```
    {
        double dist;
        getInput("distance", dist); // Get the parameter which will be defined in the xml file
        goal_.distance = dist;
    }
```

Now, we declare _dist_, which will be used to move the robot forward and backward. ```getInput("distance", dist)``` gets the parameter, which will be defined in the behavior tree xml file, and this is being set as the action request goal distance. We will cover creating the behavior tree xml file in step [TODO].

```
    static BT::PortsList providedPorts() // Define ports to get things from the xml file
    {
    return providedBasicPorts({
        BT::InputPort<double>("distance", 0.5, "Step front and back distance") // param_name, default, description
        });
    }
```

Here, we define _ports_ that allow the action node to get the parameters defined in the behavior tree xml file.

We are almost half way there!

#### Step 3 of 4:
Remember, in the beginning we said something about behavior tree ros nodes being implemented as plugins? Plugins are your friends here, so there is nothing to be afraid of.

Since we have already created our behavior tree action node class as ```StepFrontBackAction``` in ```nav2_behavior_tree/include/step_forward_back_action.hpp```, now we need to register them as a plugin.

Now, in the ```nav2_behavior_tree/plugins``` folder, create a new file named ```step_front_back_action.cpp```, and add the following code:

```
#include "nav2_behavior_tree/step_front_back_action.hpp"
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::StepFrontBackAction>("StepFrontBack"); //name it what you want to show up in the xml file
}
```

What we are doing here is registering our action node, with the behavior tree library, so that we can use our behavior tree aciton node in the ```xml``` file.

#### Step 4 of 4:

Now we need to add this plugin to the CMakeList file as shown in the example below:
```
set(bt_plugins
  ...
  step_front_back
  ...
)
```
[Where is this plugin name being taken from? I think it is the header file name.]

Then add this plugin to ```nav2_bringup/bringup/params/nav2_params.yaml``` in the scope as shown below:
```
bt_navigator:
  ros__parameters:
    plugin_lib_names:
    ...
    - nav2_step_front_back_action_bt_node
    ...
```
This is for the bt navigator to load it. Else it is build but not loaded.

#### Step 5 of 5:
Finally add it to the behavior tree xml file.
[TODO:]