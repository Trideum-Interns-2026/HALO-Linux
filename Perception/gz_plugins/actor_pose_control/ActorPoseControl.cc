/*
 * ActorPoseControl: a Gazebo (gz-sim 8) system that lets an outside program
 * drive an <actor> directly: where he stands, which way he faces, which of
 * his animations plays, and how far into it. Gazebo has no built-in way to
 * do that (FollowActor only chases a target with one fixed animation), so
 * walk_person.py sends commands here.
 *
 * Attach it inside an <actor>:
 *   <plugin filename="/abs/path/libActorPoseControl.so"
 *           name="ActorPoseControl"/>
 *
 * Topics (gz-transport, gz.msgs.StringMsg), <name> = the actor's name:
 *   /actor/<name>/cmd    in:  "x y z yaw qw qx qy qz animation anim_time_s"
 *                             (world x/y in m, z = height offset in m on top
 *                             of the spawn height, yaw = which way he faces
 *                             in rad, 0 = +X, reported back in state; qw..qz
 *                             = the hips' full orientation, applied)
 *   /actor/<name>/state  out: "x y z yaw animation anim_time_s", ~5 Hz, so a
 *                             restarted script can find him and carry on
 *                             from where he is (actors can't be removed).
 *
 * Same pose handling as gz-sim's FollowActor: the actor's Pose keeps only
 * its z, and x/y/yaw go in TrajectoryPose, which also stops any SDF script.
 * With a TrajectoryPose, Gazebo puts the root joint (hips) at it and ignores
 * the animation's own hip position and rotation (keeping the hips' rest
 * rotation), so the spawn z is the hip height, z here carries the
 * animation's hip bob/crouch on top of it, and the orientation carries his
 * facing plus the animation's hip rotation (flips, sway).
 */
#include <chrono>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

using namespace gz;
using namespace sim;

namespace
{
struct Command
{
  double x{0}, y{0}, z{0}, yaw{0};
  math::Quaterniond rot;
  std::string animation;
  double animTime{0};
};
}  // namespace

class ActorPoseControl
  : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &,
                 EntityComponentManager &_ecm, EventManager &) override
  {
    this->actor = _entity;
    auto actorComp = _ecm.Component<components::Actor>(_entity);
    if (!actorComp)
    {
      gzerr << "ActorPoseControl must be attached to an <actor>.\n";
      return;
    }
    auto name = _ecm.Component<components::Name>(_entity)->Data();

    // As in FollowActor: Pose keeps z only; x/y/yaw live in TrajectoryPose.
    math::Pose3d initial;
    if (auto poseComp = _ecm.Component<components::Pose>(_entity))
    {
      initial = poseComp->Data();
      math::Pose3d zOnly(0, 0, initial.Pos().Z(), 0, 0, 0);
      *poseComp = components::Pose(zOnly);
    }
    else
    {
      _ecm.CreateComponent(_entity, components::Pose(math::Pose3d::Zero));
    }
    Command start;
    start.x = initial.Pos().X();
    start.y = initial.Pos().Y();
    start.yaw = initial.Rot().Yaw();
    start.rot = math::Quaterniond(0, 0, start.yaw);
    if (actorComp->Data().AnimationCount() > 0)
      start.animation = actorComp->Data().AnimationByIndex(0)->Name();
    this->current = start;
    this->pending = start;

    this->node.Subscribe("/actor/" + name + "/cmd",
                         &ActorPoseControl::OnCmd, this);
    this->statePub =
        this->node.Advertise<msgs::StringMsg>("/actor/" + name + "/state");
    gzmsg << "ActorPoseControl: driving actor [" << name << "] from /actor/"
          << name << "/cmd\n";
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    std::optional<Command> cmd;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      cmd.swap(this->pending);
    }
    if (cmd)
    {
      this->current = *cmd;
      math::Pose3d pose(math::Vector3d(cmd->x, cmd->y, cmd->z), cmd->rot);
      if (auto c = _ecm.Component<components::TrajectoryPose>(this->actor))
        *c = components::TrajectoryPose(pose);
      else
        _ecm.CreateComponent(this->actor, components::TrajectoryPose(pose));
      _ecm.SetChanged(this->actor, components::TrajectoryPose::typeId,
                      ComponentState::OneTimeChange);

      if (!cmd->animation.empty())
      {
        if (auto c = _ecm.Component<components::AnimationName>(this->actor))
          *c = components::AnimationName(cmd->animation);
        else
          _ecm.CreateComponent(this->actor,
                               components::AnimationName(cmd->animation));
        _ecm.SetChanged(this->actor, components::AnimationName::typeId,
                        ComponentState::OneTimeChange);
      }

      auto t = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(cmd->animTime));
      if (auto c = _ecm.Component<components::AnimationTime>(this->actor))
        *c = components::AnimationTime(t);
      else
        _ecm.CreateComponent(this->actor, components::AnimationTime(t));
      _ecm.SetChanged(this->actor, components::AnimationTime::typeId,
                      ComponentState::OneTimeChange);
    }

    // Report where he is, ~5 Hz of wall-clock time (works while paused too).
    auto now = std::chrono::steady_clock::now();
    if (now - this->lastState > std::chrono::milliseconds(200))
    {
      this->lastState = now;
      std::ostringstream s;
      s << this->current.x << " " << this->current.y << " "
        << this->current.z << " " << this->current.yaw << " " << this->current.animation << " "
        << this->current.animTime;
      msgs::StringMsg msg;
      msg.set_data(s.str());
      this->statePub.Publish(msg);
    }
    (void)_info;
  }

private:
  void OnCmd(const msgs::StringMsg &_msg)
  {
    std::istringstream in(_msg.data());
    Command c;
    double qw, qx, qy, qz;
    if (!(in >> c.x >> c.y >> c.z >> c.yaw >> qw >> qx >> qy >> qz >> c.animation
             >> c.animTime))
    {
      gzwarn << "ActorPoseControl: ignoring malformed command ["
             << _msg.data() << "] (want \"x y z yaw qw qx qy qz animation time\")\n";
      return;
    }
    c.rot = math::Quaterniond(qw, qx, qy, qz);
    c.rot.Normalize();
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pending = c;
  }

  Entity actor{kNullEntity};
  transport::Node node;
  transport::Node::Publisher statePub;
  std::mutex mutex;
  std::optional<Command> pending;
  Command current;
  std::chrono::steady_clock::time_point lastState;
};

GZ_ADD_PLUGIN(ActorPoseControl, System, ActorPoseControl::ISystemConfigure,
              ActorPoseControl::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(ActorPoseControl, "ActorPoseControl")
