/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Link.h"
#include "Body.h"
#include "Material.h"
#include <cnoid/SceneGraph>
#include <cnoid/ValueTree>

using namespace std;
using namespace cnoid;


Link::Link()
{
    index_ = -1;
    parent_ = nullptr;
    body_ = nullptr;
    T_.setIdentity();
    Tb_.setIdentity();
    Rs_.setIdentity();
    jointType_ = FixedJoint;
    jointId_ = -1;
    actuationMode_ = StateNone;
    sensingMode_ = LINK_POSITION | JOINT_DISPLACEMENT;
    a_ = Vector3::UnitZ();
    q_ = 0.0;
    dq_ = 0.0;
    ddq_ = 0.0;
    q_target_ = 0.0;
    dq_target_ = 0.0;
    u_ = 0.0;
    v_.setZero();
    w_.setZero();
    dv_.setZero();
    dw_.setZero();
    F_ext_.setZero();
    c_.setZero();
    wc_.setZero();
    m_ = 1.0;
    I_.setIdentity();
    Jm2_ = 0.0;
    q_initial_ = 0.0;
    q_upper_ = std::numeric_limits<double>::max();
    q_lower_ = -std::numeric_limits<double>::max();
    dq_upper_ = std::numeric_limits<double>::max();
    dq_lower_ = -std::numeric_limits<double>::max();
    materialId_ = 0;
    visualShape_ = new SgPosTransform;
    collisionShape_ = new SgPosTransform;
    info_ = new Mapping;
}


Link::Link(const Link& org)
    : name_(org.name_),
      jointSpecificName_(org.jointSpecificName_)
{
    index_ = -1; // should be set by a Body object

    parent_ = nullptr;
    body_ = nullptr;

    T_ = org.T_;
    Tb_ = org.Tb_;
    
    jointType_ = org.jointType_;
    jointId_ = org.jointId_;
    actuationMode_ = org.actuationMode_;
    sensingMode_ = org.sensingMode_;
    
    a_ = org.a_;
    q_ = org.q_;
    dq_ = org.dq_;
    ddq_ = org.ddq_;
    u_ = org.u_;

    q_target_ = org.q_target_;
    dq_target_ = org.dq_target_;

    v_ = org.v_;
    w_ = org.w_;
    dv_ = org.dv_;
    dw_ = org.dw_;
    F_ext_ = org.F_ext_;
    
    c_ = org.c_;
    wc_ = org.wc_;
    m_ = org.m_;
    I_ = org.I_;

    Jm2_ = org.Jm2_;
    q_initial_ = org.q_initial_;
    q_upper_ = org.q_upper_;
    q_lower_ = org.q_lower_;
    dq_upper_ = org.dq_upper_;
    dq_lower_ = org.dq_lower_;

    materialId_ = org.materialId_;

    //! \todo add the mode for doing deep copy of the following objects
    visualShape_ = new SgPosTransform;
    org.visualShape_->copyChildrenTo(visualShape_);
    visualShape_->invalidateBoundingBox();
    collisionShape_ = new SgPosTransform;
    org.collisionShape_->copyChildrenTo(collisionShape_);
    collisionShape_->invalidateBoundingBox();
    
    info_ = org.info_;
}


Referenced* Link::doClone(CloneMap*) const
{
    return new Link(*this);
}


Link::~Link()
{
    LinkPtr link = child_;
    while(link){
        link->parent_ = nullptr;
        LinkPtr next = link->sibling_;
        link->sibling_ = nullptr;
        link = next;
    }
}


void Link::initializeState()
{
    dq_ = 0.0;
    ddq_ = 0.0;
    q_target_ = q_;
    dq_target_ = dq_;
    u_ = 0.0;
    v_.setZero();
    w_.setZero();
    dv_.setZero();
    dw_.setZero();
    F_ext_.setZero();
}


void Link::setBody(Body* newBody)
{
    if(body_ != newBody){
        setBodySub(newBody);
    }
}


void Link::setBodySub(Body* newBody)
{
    body_ = newBody;
    for(Link* link = child_; link; link = link->sibling_){
        link->setBodySub(newBody);
    }
}


bool Link::isBodyRoot() const
{
    return !parent_ || this == body_->rootLink();
}


bool Link::isStatic() const
{
    if(!isFixedJoint()){
        return false;
    }
    if(!isRoot()){
        return parent_->isStatic();
    }
    return true;
}


void Link::prependChild(Link* link)
{
    LinkPtr holder;
    if(link->parent_){
        holder = link;
        link->parent_->removeChild(link);
    }
    link->sibling_ = child_;
    child_ = link;
    link->parent_ = this;

    link->setBody(body_);
}


void Link::appendChild(Link* link)
{
    LinkPtr holder;
    if(link->parent_){
        holder = link;
        link->parent_->removeChild(link);
    }
    if(!child_){
        child_ = link;
        link->sibling_ = nullptr;
    } else {
        Link* lastChild = child_;
        while(lastChild->sibling_){
            lastChild = lastChild->sibling_;
        }
        lastChild->sibling_ = link;
        link->sibling_ = nullptr;
    }
    link->parent_ = this;

    link->setBody(body_);
}


bool Link::isOwnerOf(const Link* link) const
{
    if(link == this){
        return true;
    }
    for(const Link* owner = link->parent_; owner; owner = owner->parent_){
        if(owner == link){
            return true;
        }
    }
    return false;
}
    

/**
   A child link is removed from the link.
   If a link given by the parameter is not a child of the link, false is returned.
*/
bool Link::removeChild(Link* childToRemove)
{
    Link* link = child_;
    Link* prevSibling = nullptr;
    while(link){
        if(link == childToRemove){
            childToRemove->parent_ = nullptr;
            if(prevSibling){
                prevSibling->sibling_ = link->sibling_;
            } else {
                child_ = link->sibling_;
            }
            childToRemove->sibling_ = nullptr;
            childToRemove->setBody(0);
            return true;
        }
        prevSibling = link;
        link = link->sibling_;
    }
    return false;
}


void Link::setName(const std::string& name)
{
    if(body_){
        body_->resetLinkName(this, name);
    }
    name_ = name;
}


void Link::setJointName(const std::string& name)
{
    if(body_){
        body_->resetJointSpecificName(this, name);
    }
    jointSpecificName_ = name;
}


void Link::resetJointSpecificName()
{
    if(body_){
        body_->resetJointSpecificName(this);
    }
    jointSpecificName_.clear();
}


const std::string& Link::jointName() const
{
    if(!jointSpecificName_.empty()){
        return jointSpecificName_;
    }
    return name_;
}


std::string Link::jointTypeString(bool useUnderscore) const
{
    switch(jointType_){
    case RevoluteJoint:    return "revolute";
    case PrismaticJoint:   return "prismatic";
    case FreeJoint:        return "free";
    case FixedJoint:       return "fixed";
    case PseudoContinousTrack: {
        if(useUnderscore){
            return "pseudo_continuous_track";
        } else {
            return "pseudo continuous track";
        }
    }
    default: return "unknown";
    }
}


std::string Link::getStateModeString(short mode)
{
    if(mode == StateNone){
        return "None";
    } else {
        string s;
        for(int i=0; i < NumStateTypes; ++i){
            int modeBit = 1 << i;
            if(mode & modeBit){
                if(!s.empty()){
                    s += " + ";
                }
                switch(modeBit){
                case JointDisplacement:
                    s += "Joint Displacement";
                    break;
                case JointVelocity:
                    s += "Joint Velocity";
                    break;
                case JointAcceleration:
                    s += "Joint Acceleration";
                    break;
                case JointEffort:
                    s += "Joint Effort";
                    break;
                case LinkPosition:
                    s += "Link Position";
                    break;
                case LinkTwist:
                    s += "Link Twist";
                    break;
                case LinkExtWrench:
                    s += "Link External Wrench";
                    break;
                case LinkContactState:
                    s += "Link Contact State";
                    break;
                case JointSurfaceVelocity:
                    s += "Joint Surface Velocity";
                    break;
                default:
                    break;
                }
            }
        }
        return s;
    }
}


std::string Link::materialName() const
{
    return Material::name(materialId_);
}


void Link::setMaterial(const std::string& name)
{
    setMaterial(Material::id(name));
}


SgGroup* Link::shape() const
{
    return visualShape_;
}


SgGroup* Link::visualShape() const
{
    return visualShape_;
}


SgGroup* Link::collisionShape() const
{
    return collisionShape_;
}


bool Link::hasDedicatedCollisionShape() const
{
    if(visualShape_->numChildren() != collisionShape_->numChildren() ||
       !std::equal(visualShape_->begin(), visualShape_->end(), collisionShape_->begin())){
        return true;
    }
    return false;
}


void Link::addShapeNode(SgNode* shape, bool doNotify)
{
    visualShape_->addChild(shape, doNotify);
    collisionShape_->addChild(shape, doNotify);
}


void Link::addVisualShapeNode(SgNode* shape, bool doNotify)
{
    visualShape_->addChild(shape, doNotify);
}


void Link::addCollisionShapeNode(SgNode* shape, bool doNotify)
{
    collisionShape_->addChild(shape, doNotify);
}


void Link::removeShapeNode(SgNode* shape, bool doNotify)
{
    visualShape_->removeChild(shape, doNotify);
    collisionShape_->removeChild(shape, doNotify);
}


void Link::clearShapeNodes(bool doNotify)
{
    visualShape_->clearChildren(doNotify);
    collisionShape_->clearChildren(doNotify);
}


void Link::resetInfo(Mapping* info)
{
    info_ = info;
}


template<> double Link::info(const std::string& key) const
{
    return info_->get(key).toDouble();
}


template<> double Link::info(const std::string& key, const double& defaultValue) const
{
    double value;
    if(info_->read(key, value)){
        return value;
    }
    return defaultValue;
}


template<> bool Link::info(const std::string& key, const bool& defaultValue) const
{
    bool value;
    if(info_->read(key, value)){
        return value;
    }
    return defaultValue;
}


std::string Link::info(const std::string& key, const char* defaultValue) const
{
    string value;
    if(info_->read(key, value)){
        return value;
    }
    return defaultValue;
}


template<> void Link::setInfo(const std::string& key, const double& value)
{
    info_->write(key, value);
}


template<> void Link::setInfo(const std::string& key, const bool& value)
{
    info_->write(key, value);
}
