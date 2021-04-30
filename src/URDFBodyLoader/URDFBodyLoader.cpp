#include "URDFBodyLoader.h"

#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include <cnoid/BodyLoader>
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
using pugi::xml_node;

const char CHILD[] = "child";
const char JOINT[] = "joint";
const char LINK[] = "link";
const char NAME[] = "name";
const char PARENT[] = "parent";
const char ROBOT[] = "robot";

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

    vector<xml_node> rootLinks = findRootLinks(robot);

    if (rootLinks.size() != 1) {
        os() << "Error: " << rootLinks.size() << " root links are found.";
        return false;
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
