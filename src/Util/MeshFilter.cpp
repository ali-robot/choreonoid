/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "MeshFilter.h"
#include "MeshExtractor.h"
#include "SceneDrawables.h"
#include "IdPair.h"
#include <unordered_map>
#include <unordered_set>
#include <array>

using namespace std;
using namespace cnoid;

namespace {

const float PI = 3.14159265358979323846f;

typedef array<int, 3> FaceId;

template<class Triangle>
FaceId getFaceId(Triangle& triangle)
{
    FaceId id = { triangle[0], triangle[1], triangle[2] };
    std::sort(id.begin(), id.end());
    return id;
}

struct EdgeId : public IdPair<int>
{
    EdgeId(SgMesh::TriangleRef triangle, int index) {
        static const int offsets[][2] = { { 0, 1 }, { 1, 2 }, { 0, 2 } };
        set(triangle[offsets[index][0]], triangle[offsets[index][1]]);
    }

    int oppositeVertexIndex(SgMesh::TriangleRef triangle){
        if(id[0] == triangle[0]){
            if(id[1] == triangle[1]){
                return triangle[2];
            } else {
                return triangle[1];
            }
        } else if(id[0] == triangle[1]){
            if(id[1] == triangle[2]){
                return triangle[0];
            } else {
                return triangle[2];
            }
        } else { // id[0] == triangle[2]
            if(id[1] == triangle[0]){
                return triangle[1];
            } else {
                return triangle[0];
            }
        }
    }
};
    
}

namespace std {

template<> struct hash<FaceId>
{
    void hash_combine(std::size_t& seed, int v) const {
        std::hash<int> hasher;
        seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    }
    std::size_t operator()(const FaceId& id) const {
        std::size_t seed = 0;
        hash_combine(seed, id[0]);
        hash_combine(seed, id[1]);
        hash_combine(seed, id[2]);
        return seed;
    }
};

template<> struct hash<EdgeId>
{
    std::size_t operator()(const EdgeId& id) const {
        return std::hash<IdPair<int>>()(id);
    }
};

}

namespace cnoid {

class MeshFilterImpl
{
public:
    unique_ptr<MeshExtractor> meshExtractor;
    vector<Vector3f> faceNormals;
    vector<vector<int>> facesOfVertexMap;
    unordered_map<EdgeId, vector<int>> facesOfEdgeMap;
    vector<vector<int>> normalsOfVertexMap;
    float minCreaseAngle;
    float maxCreaseAngle;
    bool isNormalOverwritingEnabled;

    MeshFilterImpl();
    MeshFilterImpl(const MeshFilterImpl& org);
    void forAllMeshes(SgNode* node, function<void(SgMesh* mesh)> callback);
    void removeRedundantVertices(SgMesh* mesh);
    void removeRedundantFaces(SgMesh* mesh);
    void calculateFaceNormals(SgMesh* mesh, bool ignoreZeroNormals);
    void makeFacesOfVertexMap(SgMesh* mesh, bool removeSameNormalFaces = false);
    void makeFacesOfEdgeMap(SgMesh* mesh);
    void setVertexNormals(SgMesh* mesh, float creaseAngle);
};

}


MeshFilter::MeshFilter()
{
    impl = new MeshFilterImpl();
}


MeshFilterImpl::MeshFilterImpl()
{
    isNormalOverwritingEnabled = false;
    minCreaseAngle = 0.0f;
    maxCreaseAngle = PI;
}


MeshFilter::MeshFilter(const MeshFilter& org)
{
    impl = new MeshFilterImpl(*org.impl);
}


MeshFilterImpl::MeshFilterImpl(const MeshFilterImpl& org)
{
    isNormalOverwritingEnabled = org.isNormalOverwritingEnabled;
    minCreaseAngle = org.minCreaseAngle;
    maxCreaseAngle = org.maxCreaseAngle;
}


MeshFilter::~MeshFilter()
{
    delete impl;
}


void MeshFilter::setNormalOverwritingEnabled(bool on)
{
    impl->isNormalOverwritingEnabled = on;
}


void MeshFilter::setOverwritingEnabled(bool on)
{
    setNormalOverwritingEnabled(on);
}


void MeshFilter::setMinCreaseAngle(float angle)
{
    impl->minCreaseAngle = angle;
}


void MeshFilter::setMaxCreaseAngle(float angle)
{
    impl->maxCreaseAngle = angle;
}


void MeshFilterImpl::forAllMeshes(SgNode* node, function<void(SgMesh* mesh)> callback)
{
    if(!meshExtractor){
        meshExtractor.reset(new MeshExtractor);
    }
    meshExtractor->extract(node, callback);
}


void MeshFilter::integrateMeshes(SgMesh* mesh1, SgMesh* mesh2)
{
    if(!mesh1 || !mesh2 || !mesh2->hasVertices()){
        return;
    }
    auto& vertices = *mesh1->getOrCreateVertices();
    const int vertexOffset = vertices.size();
    const auto& vertices2 = *mesh2->vertices();
    vertices.resize(vertexOffset + vertices2.size());
    std::copy(vertices2.begin(), vertices2.end(), vertices.begin() + vertexOffset);

    auto& triangles = mesh1->triangleVertices();
    const auto& triangles2 = mesh2->triangleVertices();
    triangles.reserve(triangles.size() + triangles2.size());
    for(const auto& index : triangles2){
        triangles.push_back(index + vertexOffset);
    }

    if(mesh2->hasNormals() && (mesh1->hasNormals() || vertexOffset == 0)){
        auto& normals = *mesh1->getOrCreateNormals();
        const int normalOffset = normals.size();
        const auto& normals2 = *mesh2->normals();
        normals.resize(normalOffset + normals2.size());
        std::copy(normals2.begin(), normals2.end(), normals.begin() + normalOffset);

        auto& normalIndices = mesh1->normalIndices();
        auto& normalIndices2 = mesh2->normalIndices();
        normalIndices.reserve(normalIndices.size() + normalIndices2.size());
        for(const auto& index : normalIndices2){
            normalIndices.push_back(index + normalOffset);
        }
    }

    //! \todo Integrate colors and texCoords, too

    mesh1->setPrimitive(SgMesh::MESH);
}


void MeshFilter::removeRedundantVertices(SgNode* scene)
{
    impl->forAllMeshes(scene, [&](SgMesh* mesh){ impl->removeRedundantVertices(mesh); });
}


void MeshFilter::removeRedundantVertices(SgMesh* mesh)
{
    impl->removeRedundantVertices(mesh);
}


void MeshFilterImpl::removeRedundantVertices(SgMesh* mesh)
{
    const SgVertexArray& orgVertices = *mesh->vertices();
    const int numVertices = orgVertices.size();
    SgVertexArrayPtr newVerticesptr = new SgVertexArray();
    SgVertexArray& newVertices = *newVerticesptr;
    vector<int> indexMap(numVertices);

    for(int i=0; i< numVertices; ++i){
        bool found = false;
        for(int j=0; j < newVertices.size(); ++j){
            if(orgVertices[i].isApprox(newVertices[j])){
                indexMap[i] = j;
                found = true;
                break;
            }
        }
        if(!found){
            indexMap[i] = newVertices.size();
            newVertices.push_back(orgVertices[i]);
        }
    }

    if(newVertices.size() == numVertices){
        return;
    }

    mesh->setVertices(newVerticesptr);

    if(mesh->hasNormals()){
        if(mesh->normalIndices().empty()){
            mesh->normalIndices() = mesh->triangleVertices();
        }
    }

    if(mesh->hasTexCoords()){
        if(mesh->texCoordIndices().empty()){
            mesh->texCoordIndices() = mesh->triangleVertices();
        }
    }

    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
        SgMesh::TriangleRef triangle = mesh->triangle(i);
        for(int j=0; j < 3; ++j){
            triangle[j] = indexMap[triangle[j]];
        }
    }
}


void MeshFilter::removeRedundantFaces(SgNode* scene)
{
    impl->forAllMeshes(scene, [&](SgMesh* mesh){ impl->removeRedundantFaces(mesh); });
}


void MeshFilter::removeRedundantFaces(SgMesh* mesh)
{
    impl->removeRedundantFaces(mesh);
}


void MeshFilterImpl::removeRedundantFaces(SgMesh* mesh)
{
    const int numOrgTriangles = mesh->numTriangles();
    if(numOrgTriangles == 0){
        return;
    }

    SgIndexArray orgTriangles = mesh->triangleVertices();
    unordered_set<FaceId> existingFaces;
    existingFaces.reserve(numOrgTriangles);
    auto& triangles = mesh->triangleVertices();
    triangles.clear();
    
    for(int i=0; i < numOrgTriangles; ++i){
        SgMesh::ConstTriangleRef triangle(&orgTriangles[i*3]);
        if(existingFaces.insert(getFaceId(triangle)).second){
            mesh->newTriangle() = triangle;
        }            
    }
}


bool MeshFilter::generateNormals(SgMesh* mesh, float creaseAngle, bool removeRedundantVertices)
{
    if(!mesh->vertices() || mesh->triangleVertices().empty()){
        return false;
    }
    if(!impl->isNormalOverwritingEnabled && mesh->normals() && !mesh->normals()->empty()){
        return false;
    }

    if(removeRedundantVertices){
        impl->removeRedundantVertices(mesh);
    }
    impl->calculateFaceNormals(mesh, false);
    impl->makeFacesOfVertexMap(mesh, true);
    impl->setVertexNormals(mesh, creaseAngle);

    return true;
}


void MeshFilterImpl::calculateFaceNormals(SgMesh* mesh, bool ignoreZeroNormals)
{
    const SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    const int numTriangles = mesh->numTriangles();
    faceNormals.clear();
    faceNormals.reserve(numTriangles);

    for(int i=0; i < numTriangles; ++i){
        SgMesh::TriangleRef triangle = mesh->triangle(i);
        const Vector3f& v0 = vertices[triangle[0]];
        const Vector3f& v1 = vertices[triangle[1]];
        const Vector3f& v2 = vertices[triangle[2]];
        Vector3f normal((v1 - v0).cross(v2 - v0));
        // prevent NaN
        if(normal.norm() > 0.0){
            normal.normalize();
        } else {
            if(!ignoreZeroNormals){
                //! \todo remove degenerate faces
                normal = Vector3f::UnitZ();
            }
        }
        faceNormals.push_back(normal);
    }
}


void MeshFilterImpl::makeFacesOfVertexMap(SgMesh* mesh, bool removeSameNormalFaces)
{
    facesOfVertexMap.clear();
    facesOfVertexMap.resize(mesh->vertices()->size());
    const int numTriangles = mesh->numTriangles();
    
    for(int i=0; i < numTriangles; ++i){
        SgMesh::TriangleRef triangle = mesh->triangle(i);
        for(int j=0; j < 3; ++j){
            auto& faceIndicesOfVertex = facesOfVertexMap[triangle[j]];
            if(!removeSameNormalFaces){
                faceIndicesOfVertex.push_back(i);
            } else {
                /**
                   \todo Angle between adjacent edges should be taken into account
                   to generate natural normals
                */
                const auto& normal = faceNormals[i];
                bool isSameNormalFaceFound = false;
                for(size_t k=0; k < faceIndicesOfVertex.size(); ++k){
                    const auto& adjacentFaceNormal = faceNormals[faceIndicesOfVertex[k]];
                    // the same face is not appended
                    if(adjacentFaceNormal.isApprox(normal, 5.0e-4)){
                        isSameNormalFaceFound = true;
                        break;
                    }
                }
                if(!isSameNormalFaceFound){
                    faceIndicesOfVertex.push_back(i);
                }
            }
        }
    }
}
    

void MeshFilterImpl::makeFacesOfEdgeMap(SgMesh* mesh)
{
    facesOfEdgeMap.clear();
    facesOfEdgeMap.reserve(mesh->vertices()->size());
    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
        SgMesh::TriangleRef triangle = mesh->triangle(i);
        for(int j=0; j < 3; ++j){
            facesOfEdgeMap[EdgeId(triangle, j)].push_back(i);
        }
    }
}
   

void MeshFilterImpl::setVertexNormals(SgMesh* mesh, float givenCreaseAngle)
{
    const float creaseAngle = std::max(minCreaseAngle, std::min(maxCreaseAngle, givenCreaseAngle));
    
    const int numVertices = mesh->vertices()->size();
    const int numTriangles = mesh->numTriangles();

    mesh->setNormals(new SgNormalArray());
    SgNormalArray& normals = *mesh->normals();
    SgIndexArray& normalIndices = mesh->normalIndices();
    normalIndices.clear();
    normalIndices.reserve(mesh->triangleVertices().size());

    normalsOfVertexMap.clear();
    normalsOfVertexMap.resize(numVertices);

    for(int faceIndex=0; faceIndex < numTriangles; ++faceIndex){

        SgMesh::TriangleRef triangle = mesh->triangle(faceIndex);

        for(int i=0; i < 3; ++i){

            const int vertexIndex = triangle[i];
            const auto& faceIndicesOfVertex = facesOfVertexMap[vertexIndex];
            const Vector3f& currentFaceNormal = faceNormals[faceIndex];
            Vector3f normal = currentFaceNormal;
            bool normalIsFaceNormal = true;
                
            // avarage normals of the faces whose crease angle is below the 'creaseAngle' variable
            for(size_t j=0; j < faceIndicesOfVertex.size(); ++j){
                const int adjacentFaceIndex = faceIndicesOfVertex[j];
                const Vector3f& adjacentFaceNormal = faceNormals[adjacentFaceIndex];
                float cosAngle = currentFaceNormal.dot(adjacentFaceNormal)
                  / (currentFaceNormal.norm() * adjacentFaceNormal.norm());
                //prevent NaN
                if (cosAngle >  1.0) cosAngle =  1.0;
                if (cosAngle < -1.0) cosAngle = -1.0;
                const float angle = acosf(cosAngle);
                if(angle > 0.0f && angle < creaseAngle){
                    normal += adjacentFaceNormal;
                    normalIsFaceNormal = false;
                }
            }
            if(!normalIsFaceNormal){
                normal.normalize();
            }
            
            int normalIndex = -1;
            
            for(int j=0; j < 3; ++j){
                const int vertexIndex2 = triangle[j];
                const vector<int>& normalIndicesOfVertex = normalsOfVertexMap[vertexIndex2];
                for(size_t k=0; k < normalIndicesOfVertex.size(); ++k){
                    int index = normalIndicesOfVertex[k];
                    if(normals[index].isApprox(normal)){
                        normalIndex = index;
                        goto normalIndexFound;
                    }
                }
            }
            if(normalIndex < 0){
                normalIndex = normals.size();
                normals.push_back(normal);
                normalsOfVertexMap[vertexIndex].push_back(normalIndex);
            }
            
    normalIndexFound:
            normalIndices.push_back(normalIndex);
        }
    }
}
