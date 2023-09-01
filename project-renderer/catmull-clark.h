#pragma once
#include "obj-util.h"


#ifndef CAT_PARALLEL
#   define CAT_PARALLEL _Pragma("omp parallel")
#endif
#ifndef CAT_PARALLEL_FOR
#   define CAT_PARALLEL_FOR _Pragma("omp parallel for")
#endif
#ifndef CAT_PARALLEL_BARRIER
#   define CAT_PARALLEL_BARRIER _Pragma("omp barrier")
#endif
#ifndef CAT_ATOMIC
#   define CAT_ATOMIC _Pragma("omp atomic")
#endif
#ifndef CAT_CRITICAL
#   define CAT_CRITICAL _Pragma("omp critical")
#endif

namespace Catmull
{
    

    struct HalfEdge
    {
        int32_t Twin;
        int32_t Next;
        int32_t Prev;
        int32_t Vert;
        int32_t Edge;
        int32_t Face;

        // for the use of map
        bool operator<(const HalfEdge that) const {
            return memcmp((void*)this, (void*)&that, sizeof(HalfEdge)) > 0;
        };
    };

    struct Cat_Mesh
    {
        int32_t vertex_count;
        int32_t uv_count;
        int32_t halfedge_count;
        int32_t edge_count;
        int32_t face_count;

        std::vector<glm::vec3> vertices;
        std::vector<glm::vec2> uvs;
        std::vector<HalfEdge>  halfedges;

    };

    int32_t get_he_faceID(HalfEdge&);
    int32_t get_he_vertID(HalfEdge&);
    int32_t get_he_nextID(HalfEdge&);
    int32_t get_he_twinID(HalfEdge&);
    int32_t get_he_prevID(HalfEdge&);
    int32_t get_he_edgeID(HalfEdge&);
    int32_t get_faceID_by_heID(Cat_Mesh& mesh, const int32_t heID);
    int32_t get_edgeID_by_heID(Cat_Mesh& mesh, const int32_t heID);
    int32_t get_vertID_by_heID(Cat_Mesh& mesh, const int32_t heID);
    int32_t get_twinID_by_heID(Cat_Mesh& mesh, const int32_t heID);
    int32_t get_nextID_by_heID(Cat_Mesh& mesh, const int32_t heID);
    int32_t get_prevID_by_heID(Cat_Mesh& mesh, const int32_t heID);
    glm::vec3 get_vert_by_heID(Cat_Mesh&, const int32_t);
    glm::vec3 get_face_point(Cat_Mesh&, const int32_t);
    glm::vec3 get_edge_point(Cat_Mesh&, const int32_t);
    glm::vec3 get_vert_point(Cat_Mesh&, const int32_t);

	void catmull_test(Objmodel& model);

    void catmull_naive(
        Objmodel& model,
        std::vector<int>& otherHalves
    );

    void catmull_parallel(Objmodel& model, int32_t depth);

}