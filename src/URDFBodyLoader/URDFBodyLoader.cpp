#include "URDFBodyLoader.h"

#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/range/size.hpp>  // becomes STL from C++17

#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/EigenUtil>
#include <cnoid/NullOut>
#include <cnoid/SceneLoader>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <pugixml.hpp>

using namespace std;
using namespace cnoid;

namespace filesystem = cnoid::stdx::filesystem;
using fmt::format;
using pugi::xml_attribute;
using pugi::xml_node;

const char CHILD[] = "child";
const char INERTIA[] = "inertia";
const char INERTIAL[] = "inertial";
const char IXX[] = "ixx";
const char IXY[] = "ixy";
const char IXZ[] = "ixz";
const char IYY[] = "iyy";
const char IYZ[] = "iyz";
const char IZZ[] = "izz";
const char JOINT[] = "joint";
const char LINK[] = "link";
const char MASS[] = "mass";
const char NAME[] = "name";
const char ORIGIN[] = "origin";
const char PARENT[] = "parent";
const char ROBOT[] = "robot";
const char RPY[] = "rpy";
const char VALUE[] = "value";
const char XYZ[] = "xyz";

namespace {
struct Registration
{
    Registration()
    {
        BodyLoader::registerLoader({"urdf", "xacro"},
                                   []() -> AbstractBodyLoaderPtr {
                                       return make_shared<URDFBodyLoader>();
                                   });
    }
} registration;

}  // namespace

namespace cnoid {
class URDFBodyLoader::Impl
{
public:
    ostream* os_;
    ostream& os() { return *os_; }

    Impl();
    bool load(Body* body, const string& filename);

private:
    vector<xml_node> findChildrenByGrandchildAttribute(
        const xml_node& node,
        const char* child_name,
        const char* grandchild_name,
        const char* attr_name,
        const char* attr_value);
    vector<xml_node> findRootLinks(const xml_node& robot);
    bool loadLink(LinkPtr link, const xml_node& linkNode);
    bool loadInertialTag(LinkPtr& link, const xml_node& inertialNode);
    bool readOriginTag(const xml_node& originNode,
                       const string& elementName,
                       Vector3& translation,
                       Matrix3& rotation);
    void printReadingInertiaTagError(const string& attribute_name);
    bool readInertiaTag(const xml_node& inertiaNode, Matrix3& inertiaMatrix);
};
}  // namespace cnoid

URDFBodyLoader::URDFBodyLoader()
{
    impl = new Impl;
}


URDFBodyLoader::Impl::Impl()
{
    os_ = &nullout();
}


URDFBodyLoader::~URDFBodyLoader()
{
    delete impl;
}


void URDFBodyLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
}


bool URDFBodyLoader::load(Body* body, const std::string& filename)
{
    return impl->load(body, filename);
}


bool URDFBodyLoader::Impl::load(Body* body, const string& filename)
{
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());
    os() << "Load result: " << result.description() << endl;

    // checks if only one 'robot' tag exists in the URDF
    if (++doc.children(ROBOT).begin() != doc.children(ROBOT).end()) {
        os() << "Error: Multiple 'robot' tags are found.";
        return false;
    }

    // parses the robot structure ('robot' tag)
    xml_node robot = doc.child(ROBOT);
    auto linkNodes = robot.children(LINK);
    auto jointNodes = robot.children(JOINT);

    // creates a link dictionary by loading all links for tree construction
    std::unordered_map<string, LinkPtr> linkMap;
    linkMap.reserve(boost::size(linkNodes));

    for (xml_node linkNode : linkNodes) {
        LinkPtr link = new Link;
        if (!loadLink(link, linkNode)) {
            // some err msg
            return false;
        }
        linkMap.emplace(link->name(), link);
    }

    // TEST
    for (auto element : linkMap) {
        os() << "[TEST]" << endl
             << "\tName: " << element.first << endl
             << "\tMass: " << element.second->mass() << endl
             << "\tInertia: " << element.second->I().row(0) << endl
             << "\t         " << element.second->I().row(1) << endl
             << "\t         " << element.second->I().row(2) << endl
             << "\tCoM: " << element.second->centerOfMass().transpose() << endl;
    }

    return true;
}

vector<xml_node> URDFBodyLoader::Impl::findChildrenByGrandchildAttribute(
    const xml_node& node,
    const char* child_name,
    const char* grandchild_name,
    const char* attr_name,
    const char* attr_value)
{
    vector<xml_node> result;
    for (xml_node child : node.children(child_name)) {
        xml_node candidate = child.find_child_by_attribute(grandchild_name,
                                                           attr_name,
                                                           attr_value);
        if (candidate != xml_node()) {
            result.push_back(candidate);
        }
    }
    return result;
}

vector<xml_node> URDFBodyLoader::Impl::findRootLinks(const xml_node& robot)
{
    vector<xml_node> rootLinks;
    for (xml_node link : robot.children(LINK)) {
        if (findChildrenByGrandchildAttribute(robot,
                                              JOINT,
                                              CHILD,
                                              LINK,
                                              link.attribute(NAME).value())
                .size()
            == 0) {
            rootLinks.push_back(link);
        }
    }
    return rootLinks;
}


bool URDFBodyLoader::Impl::loadLink(LinkPtr link, const xml_node& linkNode)
{
    // sets name (requrired)
    std::string name(linkNode.attribute(NAME).as_string());
    if (name.empty()) {
        os() << "\033[31m Error: There exist a unnamed link.\033[m" << endl;
        return false;
    }
    link->setName(name);

    // 'inertial' (optional)
    const xml_node& inertialNode = linkNode.child(INERTIAL);
    if (inertialNode == xml_node()) {
        os() << "Debug: link '" << name << "' has no inertial data." << endl;
    } else {
        if (!loadInertialTag(link, inertialNode)) {
            os() << "Note: The above error occurs while loading link '" << name
                 << "'." << endl;
            return false;
        }
    }

    return true;
}


bool URDFBodyLoader::Impl::loadInertialTag(LinkPtr& link,
                                           const xml_node& inertialNode)
{
    // 'origin' tag
    const xml_node& originNode = inertialNode.child(ORIGIN);
    Vector3 translation;
    Matrix3 rotation;
    if (!readOriginTag(originNode, INERTIAL, translation, rotation)) {
        return false;
    }
    link->setCenterOfMass(translation);

    // 'mass' tag
    const double mass = inertialNode.child(MASS).attribute(VALUE).as_double();
    if (mass > 0.0) {
        link->setMass(mass);
    } else {
        os() << "Error: mass value is invalid or not defined." << endl;
        return false;
    }

    // 'inertia' tag
    Matrix3 inertiaMatrix = Matrix3::Identity();
    const xml_node& inertiaNode = inertialNode.child(INERTIA);
    if (inertiaNode != xml_node()) {
        if (!readInertiaTag(inertiaNode, inertiaMatrix)) {
            return false;
        }
    }
    link->setInertia(rotation * inertiaMatrix * rotation.transpose());

    return true;
}


bool URDFBodyLoader::Impl::readOriginTag(const xml_node& originNode,
                                         const string& elementName,
                                         Vector3& translation,
                                         Matrix3& rotation)
{
    const string origin_xyz_str = originNode.attribute(XYZ).as_string();
    if (origin_xyz_str.empty()) {
        translation = Vector3::Zero();
    } else {
        Vector3 origin_xyz;
        if (!toVector3(origin_xyz_str, translation)) {
            os() << "Error: origin xyz of " << elementName
                 << " is written in invalid format." << endl;
            return false;
        }
    }

    const string origin_rpy_str = originNode.attribute(RPY).as_string();
    if (origin_rpy_str.empty()) {
        rotation = Matrix3::Identity();
    } else {
        Vector3 origin_rpy;
        if (!toVector3(origin_rpy_str, origin_rpy)) {
            os() << "Error: origin rpy of " << elementName
                 << " is written in invalid format.";
            return false;
        }
        rotation = rotFromRpy(origin_rpy);
    }
    return true;
}


void URDFBodyLoader::Impl::printReadingInertiaTagError(
    const string& attribute_name)
{
    os() << "Error: " << attribute_name << " value is not defined." << endl;
    return;
}


bool URDFBodyLoader::Impl::readInertiaTag(const xml_node& inertiaNode,
                                          Matrix3& inertiaMatrix)
{
    if (inertiaNode.attribute(IXX).empty()) {
        printReadingInertiaTagError(IXX);
        return false;
    } else {
        inertiaMatrix(0, 0) = inertiaNode.attribute(IXX).as_double();
    }
    if (inertiaNode.attribute(IXY).empty()) {
        printReadingInertiaTagError(IXY);
        return false;
    } else {
        inertiaMatrix(0, 1) = inertiaNode.attribute(IXY).as_double();
        inertiaMatrix(1, 0) = inertiaMatrix(0, 1);
    }
    if (inertiaNode.attribute(IXZ).empty()) {
        printReadingInertiaTagError(IXZ);
        return false;
    } else {
        inertiaMatrix(0, 2) = inertiaNode.attribute(IXZ).as_double();
        inertiaMatrix(2, 0) = inertiaMatrix(0, 2);
    }
    if (inertiaNode.attribute(IYY).empty()) {
        printReadingInertiaTagError(IYY);
        return false;
    } else {
        inertiaMatrix(1, 1) = inertiaNode.attribute(IYY).as_double();
    }
    if (inertiaNode.attribute(IYZ).empty()) {
        printReadingInertiaTagError(IYZ);
        return false;
    } else {
        inertiaMatrix(1, 2) = inertiaNode.attribute(IYZ).as_double();
        inertiaMatrix(2, 1) = inertiaMatrix(1, 2);
    }
    if (inertiaNode.attribute(IZZ).empty()) {
        printReadingInertiaTagError(IZZ);
        return false;
    } else {
        inertiaMatrix(2, 2) = inertiaNode.attribute(IZZ).as_double();
    }

    return true;
}
