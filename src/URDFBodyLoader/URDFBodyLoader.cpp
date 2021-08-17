#include "URDFBodyLoader.h"
#include "URDFKeywords.h"

#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/range/size.hpp>  // becomes STL from C++17

#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/EigenUtil>
#include <cnoid/MeshGenerator>
#include <cnoid/NullOut>
#include <cnoid/SceneLoader>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <pugixml.hpp>

using namespace cnoid;

namespace filesystem = cnoid::stdx::filesystem;
using fmt::format;
using pugi::xml_attribute;
using pugi::xml_node;
using std::endl;
using std::string;
using std::vector;

namespace {
struct Registration
{
    Registration()
    {
        BodyLoader::registerLoader({"urdf", "xacro"},
                                   []() -> AbstractBodyLoaderPtr {
                                       return std::make_shared<URDFBodyLoader>();
                                   });
    }
} registration;


class ROSPackageSchemeHandler
{
    vector<string> packagePaths;

public:
    ROSPackageSchemeHandler()
    {
        const char* str = getenv("ROS_PACKAGE_PATH");
        if (str) {
            do {
                const char* begin = str;
                while (*str != ':' && *str)
                    str++;
                packagePaths.push_back(string(begin, str));
            } while (0 != *str++);
        }
    }

    string operator()(const string& path, std::ostream& os)
    {
        const string prefix = "package://";
        if (!(path.size() >= prefix.size()
              && std::equal(prefix.begin(), prefix.end(), path.begin()))) {
            return path;
        }

        filesystem::path filepath(fromUTF8(path.substr(prefix.size())));
        auto iter = filepath.begin();
        if (iter == filepath.end()) {
            return string();
        }

        filesystem::path directory = *iter++;
        filesystem::path relativePath;
        while (iter != filepath.end()) {
            relativePath /= *iter++;
        }

        bool found = false;
        filesystem::path combined;

        for (auto element : packagePaths) {
            filesystem::path packagePath(element);
            combined = packagePath / filepath;
            if (exists(combined)) {
                found = true;
                break;
            }
            combined = packagePath / relativePath;
            if (exists(combined)) {
                found = true;
                break;
            }
        }

        if (found) {
            return toUTF8(combined.string());
        } else {
            os << format("\"{}\" is not found in the ROS package directories.",
                         path)
               << endl;
            return string();
        }
    }
};

}  // namespace

namespace cnoid {
class URDFBodyLoader::Impl
{
public:
    std::ostream* os_;
    std::ostream& os() { return *os_; }

    Impl();
    bool load(Body* body, const string& filename);

private:
    int jointCounter_;
    SceneLoader sceneLoader_;
    ROSPackageSchemeHandler ROSPackageSchemeHandler_;

    vector<LinkPtr> findRootLinks(
        const std::unordered_map<string, LinkPtr>& linkMap);
    bool loadLink(LinkPtr link, const xml_node& linkNode);
    bool loadInertialTag(LinkPtr& link, const xml_node& inertialNode);
    bool loadVisualTag(LinkPtr& link, const xml_node& visualNode);
    bool loadCollisionTag(LinkPtr& link, const xml_node& collisionNode);
    bool readOriginTag(const xml_node& originNode,
                       const string& elementName,
                       Vector3& translation,
                       Matrix3& rotation);
    void printReadingInertiaTagError(const string& attribute_name);
    bool readInertiaTag(const xml_node& inertiaNode, Matrix3& inertiaMatrix);
    bool readGeometryTag(const xml_node& geometryNode, SgNodePtr& mesh);
    bool loadJoint(std::unordered_map<string, LinkPtr>& linkMap,
                   const xml_node& jointNode);
};
}  // namespace cnoid

URDFBodyLoader::URDFBodyLoader()
{
    impl = new Impl;
}


URDFBodyLoader::Impl::Impl()
{
    os_ = &nullout();
    jointCounter_ = 0;
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
    const pugi::xml_parse_result result = doc.load_file(filename.c_str());
    if (!result) {
        os() << "Error: parsing XML failed: " << result.description() << endl;
    }

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

    // loads all links
    for (xml_node linkNode : linkNodes) {
        LinkPtr link = new Link;
        if (!loadLink(link, linkNode)) {
            return false;
        }
        linkMap.emplace(link->name(), link);
    }

    // loads all joints with creating a link tree
    for (xml_node jointNode : jointNodes) {
        if (!loadJoint(linkMap, jointNode)) {
            // TODO: print some err msg
            return false;
        }
    }

    // finds the root link
    vector<LinkPtr> rootLinks = findRootLinks(linkMap);
    if (rootLinks.empty()) {
        os() << "Error: no root link is found." << endl;
    } else if (rootLinks.size() > 1) {
        os() << "Error: multiple root links are found." << endl;
    }
    body->setRootLink(rootLinks.at(0));

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


vector<LinkPtr> URDFBodyLoader::Impl::findRootLinks(
    const std::unordered_map<string, LinkPtr>& linkMap)
{
    vector<LinkPtr> rootLinks;
    for (auto mapElement : linkMap) {
        const LinkPtr link = mapElement.second;
        if (link->parent() == nullptr) {
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

    // 'visual' (optional)
    const xml_node& visualNode = linkNode.child(VISUAL);
    if (visualNode.empty()) {
        os() << "Debug: link '" << name << "' has no visual data." << endl;
    } else {
        if (!loadVisualTag(link, visualNode)) {
            os() << "Note: The above error occurs while loading link '" << name
                 << "'." << endl;
            return false;
        }
    }

    // 'collision' (optional)
    const xml_node& collisionNode = linkNode.child(COLLISION);
    if (collisionNode.empty()) {
        os() << "Debug: link '" << name << "' has no collision data." << endl;
    } else {
        if (!loadCollisionTag(link, collisionNode)) {
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


bool URDFBodyLoader::Impl::loadVisualTag(LinkPtr& link,
                                         const xml_node& visualNode)
{
    // 'origin' tag
    const xml_node& originNode = visualNode.child(ORIGIN);
    Vector3 translation = Vector3::Zero();
    Matrix3 rotation = Matrix3::Identity();
    if (!readOriginTag(originNode, INERTIAL, translation, rotation)) {
        return false;
    }
    Isometry3 originalPose;
    originalPose.linear() = rotation;
    originalPose.translation() = translation;

    // 'geometry' tag
    const xml_node& geometryNode = visualNode.child(GEOMETRY);
    if (geometryNode.empty()) {
        os() << "Error: Visual geometry is not found." << endl;
        return false;
    }

    SgNodePtr mesh = new SgNode;
    if (!readGeometryTag(geometryNode, mesh)) {
        os() << "Error: Failed to load visual geometry" << endl;
    }
    SgPosTransformPtr transformation = new SgPosTransform(originalPose);
    transformation->addChild(mesh);
    link->addVisualShapeNode(transformation);

    // TODO: 'material' tag

    return true;
}


bool URDFBodyLoader::Impl::loadCollisionTag(LinkPtr& link,
                                            const xml_node& collisionNode)
{
    // 'origin' tag
    const xml_node& originNode = collisionNode.child(ORIGIN);
    Vector3 translation = Vector3::Zero();
    Matrix3 rotation = Matrix3::Identity();
    if (!readOriginTag(originNode, INERTIAL, translation, rotation)) {
        return false;
    }
    Isometry3 originalPose;
    originalPose.linear() = rotation;
    originalPose.translation() = translation;

    // 'geometry' tag
    const xml_node& geometryNode = collisionNode.child(GEOMETRY);
    if (geometryNode.empty()) {
        os() << "Error: Collision geometry is not found." << endl;
        return false;
    }

    SgNodePtr mesh = new SgNode;
    if (!readGeometryTag(geometryNode, mesh)) {
        os() << "Error: Failed to load collision geometry" << endl;
    }
    SgPosTransformPtr transformation = new SgPosTransform(originalPose);
    transformation->addChild(mesh);
    link->addCollisionShapeNode(transformation);

    return true;
}


bool URDFBodyLoader::Impl::readOriginTag(const xml_node& originNode,
                                         const string& parentName,
                                         Vector3& translation,
                                         Matrix3& rotation)
{
    const string origin_xyz_str = originNode.attribute(XYZ).as_string();
    if (origin_xyz_str.empty()) {
        translation = Vector3::Zero();
    } else {
        Vector3 origin_xyz;
        if (!toVector3(origin_xyz_str, translation)) {
            os() << "Error: origin xyz of " << parentName
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
            os() << "Error: origin rpy of " << parentName
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


bool URDFBodyLoader::Impl::readGeometryTag(const xml_node& geometryNode,
                                           SgNodePtr& mesh)
{
    if (boost::size(geometryNode.children()) < 1) {
        os() << "Error: no geometry is found." << endl;
    } else if (boost::size(geometryNode.children()) > 1) {
        os() << "Error: one link can have only one geometry." << endl;
    }

    MeshGenerator meshGenerator;
    SgShapePtr shape = new SgShape;
    // const xml_node& elementNode = geometryNode.first_child();

    if (!geometryNode.child(BOX).empty()) {
        Vector3 size = Vector3::Zero();
        if (!toVector3(geometryNode.child(BOX).attribute(SIZE).as_string(),
                       size)) {
            os() << "Error: box size is written in invalid format." << endl;
        }

        shape->setMesh(meshGenerator.generateBox(size));
        mesh = shape;
    } else if (!geometryNode.child(CYLINDER).empty()) {
        if (geometryNode.child(CYLINDER).attribute(RADIUS).empty()) {
            os() << "Error: cylinder radius is not defined." << endl;
            return false;
        }
        if (geometryNode.child(CYLINDER).attribute(LENGTH).empty()) {
            os() << "Error: cylinder length is not defined." << endl;
            return false;
        }
        const double radius
            = geometryNode.child(CYLINDER).attribute(RADIUS).as_double();
        const double length
            = geometryNode.child(CYLINDER).attribute(LENGTH).as_double();

        shape->setMesh(meshGenerator.generateCylinder(radius, length));
        mesh = shape;
    } else if (!geometryNode.child(SPHERE).empty()) {
        if (geometryNode.child(SPHERE).attribute(RADIUS).empty()) {
            os() << "Error: sphere radius is not defined." << endl;
            return false;
        }
        const double radius
            = geometryNode.child(SPHERE).attribute(RADIUS).as_double();

        shape->setMesh(meshGenerator.generateSphere(radius));
        mesh = shape;
    } else if (!geometryNode.child(MESH).empty()) {
        if (geometryNode.child(MESH).attribute(FILENAME).empty()) {
            os() << "Error: mesh file is not specified." << endl;
            return false;
        }

        // loads a mesh file
        const string filename
            = geometryNode.child(MESH).attribute(FILENAME).as_string();
        bool isSupportedFormat = false;
        mesh = sceneLoader_.load(ROSPackageSchemeHandler_(filename, os()),
                                 isSupportedFormat);
        if (!isSupportedFormat) {
            os() << "Error: format of the specified mesh file '" << filename
                 << "' is not supported." << endl;
            return false;
        }

        // scales the mesh
        if (!geometryNode.child(MESH).attribute(SCALE).empty()) {
            Vector3 scale = Vector3::Ones();
            if (!toVector3(geometryNode.child(MESH).attribute(SCALE).as_string(),
                           scale)) {
                os() << "Error: mesh scale is written in invalid format."
                     << endl;

                return false;
            }

            SgScaleTransformPtr scaler = new SgScaleTransform;
            scaler->setScale(scale);
            scaler->addChild(mesh);
            mesh = scaler;
        }
    } else {
        os() << "Error: unsupported geometry "
             << geometryNode.first_child().name() << " is described." << endl;
        return false;
    }

    return true;
}

bool URDFBodyLoader::Impl::loadJoint(
    std::unordered_map<string, LinkPtr>& linkMap, const xml_node& jointNode)
{
    // 'name' attribute (required)
    if (jointNode.attribute(NAME).empty()) {
        os() << "Error: There exist a unnamed joint." << endl;
        return false;
    }
    const string jointName = jointNode.attribute(NAME).as_string();

    // 'parent' tag (required)
    if (jointNode.child(PARENT).attribute(LINK).empty()) {
        os() << "Error: joint '" << jointName << "' has no parent." << endl;
        return false;
    }
    const string parentName = jointNode.child(PARENT).attribute(LINK).as_string();
    LinkPtr parent = (*linkMap.find(parentName)).second;

    // 'child' tag (required)
    if (jointNode.child(CHILD).attribute(LINK).empty()) {
        os() << "Error: joint '" << jointName << "' has no child." << endl;
        return false;
    }
    const string childName = jointNode.child(CHILD).attribute(LINK).as_string();
    LinkPtr child = (*linkMap.find(childName)).second;

    parent->appendChild(child);
    child->setParent(parent);
    child->setJointName(jointName);

    // 'type' attribute (required)
    if (jointNode.attribute(TYPE).empty()) {
        os() << "Error: type of joint '" << jointName << "' is not defined."
             << endl;
        return false;
    }
    const string jointType = jointNode.attribute(TYPE).as_string();
    if (jointType == REVOLUTE || jointType == CONTINUOUS) {
        child->setJointType(Link::RevoluteJoint);
        child->setJointId(jointCounter_++);
    } else if (jointType == PRISMATIC) {
        child->setJointType(Link::PrismaticJoint);
        child->setJointId(jointCounter_++);
    } else if (jointType == FIXED) {
        child->setJointType(Link::FixedJoint);
    }

    // 'origin' tag (optional)
    if (!jointNode.child(ORIGIN).empty()) {
        Vector3 translation;
        Matrix3 rotation;
        if (!readOriginTag(jointNode.child(ORIGIN),
                           JOINT,
                           translation,
                           rotation)) {
            return false;
        }
        child->setOffsetTranslation(translation);
        child->setOffsetRotation(rotation);
    }

    // 'axis' tag (optional)
    if (jointNode.child(AXIS).empty()) {
        child->setJointAxis(Vector3::UnitX());
    } else {
        if (jointNode.child(AXIS).attribute(XYZ).empty()) {
            os() << "Error: axis of joint '" << jointName
                 << "'is not defined while 'axis' tag is written.";
            return false;
        }

        Vector3 axis = Vector3::UnitX();
        if (!toVector3(jointNode.child(AXIS).attribute(XYZ).as_string(), axis)) {
            os() << "Error: axis of joint '" << jointName
                 << "' is written in invalid format." << endl;
            return false;
        }
        child->setJointAxis(axis);
    }

    // 'limit' tag (partially required)
    if (jointNode.child(LIMIT).empty()) {
        if (jointType == REVOLUTE || jointType == PRISMATIC) {
            os() << "Error: limit of joint '" << jointName
                 << "' is not defined." << endl;
            return false;
        }
    } else {
        const xml_node& limitNode = jointNode.child(LIMIT);
        // 'lower' and 'upper' attributes (optional, default: 0.0)
        if (jointType == REVOLUTE || jointType == PRISMATIC) {
            double lower = 0.0, upper = 0.0;
            if (!limitNode.attribute(LOWER).empty()) {
                lower = limitNode.attribute(LOWER).as_double();
            }
            if (!limitNode.attribute(UPPER).empty()) {
                upper = limitNode.attribute(UPPER).as_double();
            }
            child->setJointRange(lower, upper);
        }
        // 'velocity' and 'effort' attributes (required)
        if (jointType == REVOLUTE || jointType == PRISMATIC
            || jointType == CONTINUOUS) {
            if (limitNode.attribute(VELOCITY).empty()) {
                os() << "Error: velocity limit of joint '" << jointName
                     << "' is not defined." << endl;
                return false;
            }
            const double velocityLimit = limitNode.attribute(VELOCITY)
                                             .as_double();
            if (velocityLimit < 0.0) {
                os() << "Error: velocity limit of joint '" << jointName
                     << "' have to be positive." << endl;
                return false;
            }
            child->setJointVelocityRange(-velocityLimit, velocityLimit);

            // does choreonoid have effort limit functions?
            //
            // if (limitNode.attribute(EFFORT).empty()) {
            //     os() << "Error: effort limit of joint '" << jointName
            //          << "' is not defined." << endl;
            //     return false;
            // }
            // const double effortLimit = limitNode.attribute(EFFORT)
            //                                  .as_double();
            // if (effortLimit < 0.0) {
            //     os() << "Error: effort limit of joint '" << jointName
            //          << "' have to be positive." << endl;
            //     return false;
            // }
        }
    }

    // 'dynamics' tag (not supported in choreonoid)
    if (!jointNode.child(DYNAMICS).empty()) {
        os() << "Warning: 'dynamics' tag is currently not supported." << endl;
    }

    // 'mimic' tag (not supported in choreonoid)
    if (!jointNode.child(MIMIC).empty()) {
        os() << "Warning: mimic joint is currently not supported." << endl;
    }
    return true;
}
