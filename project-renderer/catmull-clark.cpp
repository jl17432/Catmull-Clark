#include "catmull-clark.h"
#include <iostream>
#include <map>
#include <chrono>


#include <omp.h>


// helper func：calculate the sum of vec3s
glm::vec3 vec3SumHelper(const glm::vec3& val) {
    return val;
}

template<typename... Args>
glm::vec3 vec3SumHelper(const glm::vec3& first, Args... rest) {
    return first + vec3SumHelper(rest...);
}

// calculate the average of vec3s
template<typename... Args>
glm::vec3 vec3Average(const glm::vec3& first, Args... rest) {
    int numArgs = sizeof...(Args) + 1;
    glm::vec3 sum = vec3SumHelper(first, rest...);
    return sum / static_cast<float>(numArgs);


}



namespace Catmull
{

    void catmull_test(Objmodel& myModel)
	{
        std::vector<int> otherHalves;

        ObjUtils::remove_duplicate_pos(myModel);


        // ====================================================================================================
        // ====================================================================================================

        /*auto start_time = std::chrono::high_resolution_clock::now();
        ObjUtils::generate_otherHalves(myModel.indices, otherHalves, 4);
        catmull_naive(myModel, otherHalves);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        std::cout << "naive Catmull-Clark subdivision cost: " << duration << " m/s time\n";


        std::ofstream outfile;
        outfile.open("../output-naive.obj", std::ios::out | std::ios::trunc);
        myModel.writeObj(outfile);
        outfile.close();*/


        // ====================================================================================================
        // ====================================================================================================

        int desired_num_threads = 1;
        omp_set_num_threads(desired_num_threads);
        
        auto start_time = std::chrono::high_resolution_clock::now();
        catmull_parallel(myModel, 1);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        std::cout << "parallel Catmull-Clark subdivision cost: " << duration << " (ms) time\n";


        std::ofstream outfile;
        outfile.open("../output-parallel.obj", std::ios::out | std::ios::trunc);
        myModel.writeObj(outfile);
        outfile.close();
	}

    /* =============================================================================
    * The following funcs performs a single-thread naive catmull-clark subdivision
    *
    */
    void geverate_new_model_naive(
        Objmodel& model,
        std::vector<glm::vec3>& face_points,
        std::vector<glm::vec3>& edge_points,
        std::map<unsigned int, unsigned int> edge_pt_map
    )
    {
        unsigned int numOfFace = model.indices.size() / 4.f;
        unsigned int fp_offset = model.vertices.size();
        unsigned int ep_offset = fp_offset + face_points.size();

        std::vector<unsigned int> new_indices;

        for (unsigned int face = 0; face < numOfFace; ++face)
        {
            int vertex_id = face * 4;

            new_indices.push_back(model.indices[vertex_id]);
            new_indices.push_back(edge_pt_map[vertex_id] + ep_offset);
            new_indices.push_back(face + fp_offset);
            auto temp_point = edge_pt_map[vertex_id + 3] + ep_offset;
            new_indices.push_back(temp_point);

            vertex_id += 1;
            new_indices.push_back(model.indices[vertex_id]);
            new_indices.push_back(edge_pt_map[vertex_id] + ep_offset);
            new_indices.push_back(face + fp_offset);
            new_indices.push_back(edge_pt_map[vertex_id - 1] + ep_offset);

            vertex_id += 1;
            new_indices.push_back(model.indices[vertex_id]);
            new_indices.push_back(edge_pt_map[vertex_id] + ep_offset);
            new_indices.push_back(face + fp_offset);
            new_indices.push_back(edge_pt_map[vertex_id - 1] + ep_offset);

            vertex_id += 1;
            new_indices.push_back(model.indices[vertex_id]);
            new_indices.push_back(edge_pt_map[vertex_id] + ep_offset);
            new_indices.push_back(face + fp_offset);
            new_indices.push_back(edge_pt_map[vertex_id - 1] + ep_offset);
        }

        model.indices = new_indices;
        model.vertices.insert(model.vertices.end(), face_points.begin(), face_points.end());
        model.vertices.insert(model.vertices.end(), edge_points.begin(), edge_points.end());
        return;
    }


    void catmull_naive(
        Objmodel& model,
        std::vector<int>& otherHalves
    )
    {
        std::vector<glm::vec3> face_mid_points;
        std::vector<glm::vec3> edge_mid_points;
        std::vector<glm::vec3> new_edge_points;
        std::map<unsigned int, std::vector<unsigned int>> v2f_map;


        std::map<unsigned int, unsigned int> edge_mp_map;
        std::map<unsigned int, unsigned int> edge_pt_map;

        unsigned int num_of_original_vertices = model.vertices.size();

        for (unsigned int edge = 0; edge < model.indices.size(); edge += 4)
        {
            // face points ==> 
                //      * after N = num of original vertices, has size of num of faces (== num of original vertices / 4)
                //      * can be indexed by [ID of face]
            {
                glm::vec3 v0, v1, v2, v3;
                v0 = model.vertices[model.indices[edge]];
                v1 = model.vertices[model.indices[edge + 1]];
                v2 = model.vertices[model.indices[edge + 2]];
                v3 = model.vertices[model.indices[edge + 3]];
                                
                face_mid_points.push_back(vec3Average(v0, v1, v2, v3));
            }


            // edge mid points ==>
            //      * will be used for calculating edge points and the new vertices positions
            //      * pushed into the vector by the order of: 
            //              - v0v1_mid_point
            //              - v1v2_mid_point
            //              - v2v3_mid_point
            //              - v3v0_mid_point
            //      * can be indexed by [ID of edge]
            {
                for (unsigned int curr_edge = edge; curr_edge < edge + 4; ++curr_edge)
                {
                    // first update v2f_map, match the face id to all the vertices on this face
                    v2f_map[model.indices[curr_edge]].push_back(edge / 4);

                    // calculate mid points
                    auto iter = edge_mp_map.find(otherHalves[curr_edge]);

                    
                    if (iter != edge_mp_map.end())      // means this edge's otherhalf has calculated the mid point for it
                    {
                        edge_mp_map[curr_edge] = edge_mp_map[otherHalves[curr_edge]];
                    }
                    
                    else       // means this edge's mid point is a new vertex, calculate and record it
                    {
                        glm::vec3 vhead = model.vertices[model.indices[curr_edge]];
                        glm::vec3 vtail = (curr_edge == (edge + 3)) ? model.vertices[model.indices[edge]] : model.vertices[model.indices[curr_edge + 1]];
                        edge_mid_points.push_back(vec3Average(vhead, vtail));
                        edge_mp_map[curr_edge] = edge_mid_points.size() - 1;
                        edge_mp_map[otherHalves[curr_edge]] = edge_mid_points.size() - 1;
                    }
                }
            }            
        }

        // new edge points 
        for (unsigned int edge = 0; edge < model.indices.size(); ++edge)
        {
            auto iter = edge_pt_map.find(otherHalves[edge]);
            if (iter == edge_pt_map.end())
            {
                glm::vec3 vhead = model.vertices[model.indices[edge]];
                glm::vec3 vtail = (edge % 4 == 3) ? model.vertices[model.indices[edge - 3]] : model.vertices[model.indices[edge + 1]];
                glm::vec3 average_ep = vec3Average(vhead, vtail);

                glm::vec3 facepoint = face_mid_points[edge / 4];
                glm::vec3 vneighbour = face_mid_points[otherHalves[edge] / 4];
                glm::vec3 average_fp = vec3Average(facepoint, vneighbour);

                new_edge_points.push_back(vec3Average(average_ep, average_fp));
                edge_pt_map[edge] = new_edge_points.size() - 1;
                edge_pt_map[otherHalves[edge]] = new_edge_points.size() - 1;
            }            
        }


        // update original vertices
        for (unsigned int vertex = 0; vertex < model.vertices.size(); ++vertex)
        {
            //std::cout << vertex << ": " << model.vertices[vertex].x << " " << model.vertices[vertex].y << " " << model.vertices[vertex].z << std::endl;
            unsigned int valence = v2f_map[vertex].size();

            glm::vec3 avg_fp = glm::vec3(0,0,0), avg_ep = glm::vec3(0,0,0);
            for (auto face : v2f_map[vertex])
            {
                // average face point
                avg_fp = avg_fp + face_mid_points[face];

                int edge_in = -1, edge_out = -1;
                for (unsigned int i = 0; i < 4; ++i)
                {
                    if (model.vertices[model.indices[face * 4 + i]] == model.vertices[vertex])
                    {
                        edge_out = face * 4 + i;
                        edge_in = (i == 0) ? edge_out + 3 : edge_out - 1;
                        break;
                    }
                }
                
                avg_ep = avg_ep + edge_mid_points[edge_mp_map[edge_in]] + edge_mid_points[edge_mp_map[edge_out]];
            }
            avg_fp = avg_fp / (float) valence;
            avg_ep = avg_ep / (float) (valence * 2);

            model.vertices[vertex] = (avg_fp + avg_ep * 2.f + model.vertices[vertex] * (float)(valence - 3)) / (float) valence;
        }

        geverate_new_model_naive(model, face_mid_points, new_edge_points, edge_pt_map);
    }





    /*=======================================================================
    * The funcs below will perform Catmull-Clark subdivision in parallel 
    *   following the half-edge rule proposed by J. Dupuy, K. Vanhoey.
    * 
    */




    // This func will generate all the halfedge structures of the mesh
    // it will leave the twin term and the edge term to -1 for all halfedges, where
    // they will be computed in the next procedure.
    void generate_halfedge_without_twins(Objmodel& model, Cat_Mesh& catMesh)
    {
        std::vector<HalfEdge> halfedges;
       
        for (int32_t index = 0; index < model.indices.size(); ++index)
        {
            int32_t next = (index % 4 == 3) ? index - 3 : index + 1;
            int32_t prev = (index % 4 == 0) ? index + 3 : index - 1;
            int32_t vert = model.indices[index];
            int32_t face = index / 4.f;
            HalfEdge halfedge{ -1, next, prev, vert, -1, face };
            halfedges.push_back(halfedge);
        }
        catMesh.halfedges = halfedges; 
        catMesh.halfedge_count = halfedges.size();

        catMesh.vertices = model.vertices;
        catMesh.vertex_count = model.vertices.size();

        catMesh.face_count = catMesh.halfedge_count / 4.f;
    }

    // This func will compute twin for all halfedges.
    void generate_twins(Objmodel& model, Cat_Mesh& catMesh)
    {
        std::map<int32_t, int32_t> dictionary;
        CAT_PARALLEL_FOR
            for (int32_t halfedge = 0; halfedge < catMesh.halfedges.size(); ++halfedge)
            {                    
                int32_t key = get_he_vertID(catMesh.halfedges[halfedge]) + 
                    catMesh.vertex_count * get_he_vertID(catMesh.halfedges[get_he_nextID(catMesh.halfedges[halfedge])]);
                CAT_CRITICAL
                { dictionary[key] = halfedge; }
            }
        CAT_PARALLEL_BARRIER
            
        CAT_PARALLEL_FOR
            for (int32_t halfedge = 0; halfedge < catMesh.halfedges.size(); ++halfedge)
            {
                int32_t key = catMesh.vertex_count * get_he_vertID(catMesh.halfedges[halfedge]) +
                    get_he_vertID(catMesh.halfedges[get_he_nextID(catMesh.halfedges[halfedge])]);
                catMesh.halfedges[halfedge].Twin = dictionary[key];
            }
        CAT_PARALLEL_BARRIER        
    }

    // This func will compute all the edges (undirected edges, not halfedges)
    void generate_edges(Objmodel& model, Cat_Mesh& catMesh)
    {
        std::map<std::pair<int32_t, int32_t>, int32_t> edge_dictionary;
        int32_t edge_count = 0;

        for (int32_t halfedge = 0; halfedge < catMesh.halfedge_count; ++halfedge)
        {
            std::pair < int32_t, int32_t> edge = { std::min(catMesh.halfedges[halfedge].Vert, catMesh.halfedges[catMesh.halfedges[halfedge].Next].Vert), 
                std::max(catMesh.halfedges[halfedge].Vert, catMesh.halfedges[catMesh.halfedges[halfedge].Next].Vert) };

            if (edge_dictionary.count(edge) < 1)
            {
                edge_dictionary[edge] = edge_count;
                ++edge_count;
            }
        }
        catMesh.edge_count = edge_count;

        CAT_PARALLEL_FOR
            for (int32_t halfedge = 0; halfedge < catMesh.halfedges.size(); ++halfedge)
            {
                std::pair < int32_t, int32_t> edge = { std::min(catMesh.halfedges[halfedge].Vert, catMesh.halfedges[catMesh.halfedges[halfedge].Next].Vert),
                std::max(catMesh.halfedges[halfedge].Vert, catMesh.halfedges[catMesh.halfedges[halfedge].Next].Vert) };
                catMesh.halfedges[halfedge].Edge = edge_dictionary[edge];
            }
        CAT_PARALLEL_BARRIER
    }


    /*=====================================================================
    * The Catmull-Clark subdivision funcs, calculating new vertices
    * 
    */
    Cat_Mesh obj_2_catmesh(Objmodel& model)
    {
        Cat_Mesh result; 
        generate_halfedge_without_twins(model, result);
        generate_twins(model, result);
        generate_edges(model, result);

        return result;
    }

    void compute_face_point_parallel(Cat_Mesh& mesh)
    {
        const int32_t old_vert_size = mesh.vertex_count + mesh.vertex_count;
        const int32_t new_vert_size = old_vert_size + mesh.face_count;
        const int32_t halfedge_size = mesh.halfedge_count;

        //mesh.vertices.reserve(new_vert_size);
        mesh.vertices.resize(new_vert_size);

        CAT_PARALLEL_FOR
            for (int32_t halfedge = 0; halfedge < halfedge_size; ++halfedge)
            {
                float vert_in_face = 1;
                const int32_t faceID = get_he_faceID(mesh.halfedges[halfedge]);
                int32_t curr_vertID = old_vert_size + faceID;

                for (auto curr_halfedgeID = get_he_nextID(mesh.halfedges[halfedge]);
                    curr_halfedgeID != halfedge;
                    curr_halfedgeID = get_he_nextID(mesh.halfedges[curr_halfedgeID]))
                {
                    ++vert_in_face;                    
                }

                CAT_CRITICAL
                {
                    mesh.vertices[curr_vertID] = mesh.vertices[curr_vertID] + get_vert_by_heID(mesh, halfedge) / vert_in_face;
                }
            }
        CAT_PARALLEL_BARRIER
    }

    void compute_edge_point_parallel(Cat_Mesh& mesh)
    {
        const int32_t old_vert_size = mesh.vertex_count + mesh.vertex_count + mesh.face_count;
        const int32_t new_vert_size = old_vert_size + mesh.edge_count;
        const int32_t halfedge_size = mesh.halfedge_count;
        
        //mesh.vertices.reserve(new_vert_size);
        mesh.vertices.resize(new_vert_size);

        CAT_PARALLEL_FOR
            for (int32_t halfedge = 0; halfedge < halfedge_size; ++halfedge)
            {
                const int32_t faceID = get_he_faceID(mesh.halfedges[halfedge]);
                const int32_t edgeID = get_he_edgeID(mesh.halfedges[halfedge]);
                const int32_t twinID = get_he_twinID(mesh.halfedges[halfedge]);

                int32_t curr_vertID = old_vert_size + edgeID;

                if (twinID != -1)    // not boundry
                {
                    glm::vec3 temp = ( get_vert_by_heID(mesh, halfedge) + get_face_point(mesh, faceID) ) * 0.25f;

                    CAT_CRITICAL
                    mesh.vertices[curr_vertID] = mesh.vertices[curr_vertID] + temp;
                }
                else    // boundary
                {
                    const int32_t nextID = get_he_nextID(mesh.halfedges[halfedge]);

                    glm::vec3 temp = ( get_vert_by_heID(mesh, halfedge) + get_vert_by_heID(mesh, nextID) ) * 0.5f;

                    CAT_CRITICAL
                    {
                        mesh.vertices[curr_vertID] = mesh.vertices[curr_vertID] + temp;
                    }
                }
            }
        CAT_PARALLEL_BARRIER
    }

    void compute_vert_point_parallel(Cat_Mesh& mesh)
    {
        const int32_t old_vert_size = mesh.vertex_count;
        const int32_t halfedge_size = mesh.halfedge_count;

        CAT_PARALLEL_FOR
            for (int32_t halfedge = 0; halfedge < halfedge_size; ++halfedge)
            {
                const int32_t faceID = get_he_faceID(mesh.halfedges[halfedge]);
                const int32_t edgeID = get_he_edgeID(mesh.halfedges[halfedge]);
                const int32_t twinID = get_he_twinID(mesh.halfedges[halfedge]);
                const int32_t vertID = get_he_vertID(mesh.halfedges[halfedge]);
                const int32_t nextID = get_he_nextID(mesh.halfedges[halfedge]);
                const int32_t prevID = get_he_prevID(mesh.halfedges[halfedge]);

                int32_t curr_vertID = old_vert_size + vertID;

                int32_t valence = 1;
                int32_t forward_iter, backward_iter;

                for (forward_iter = get_he_twinID(mesh.halfedges[prevID]);
                    forward_iter >= 0 && forward_iter != halfedge;
                    forward_iter = get_he_twinID(mesh.halfedges[forward_iter])
                    )
                {                  
                    ++valence;
                    forward_iter = get_he_prevID(mesh.halfedges[forward_iter]);
                }

                for (backward_iter = get_he_twinID(mesh.halfedges[nextID]);
                    forward_iter < 0 && backward_iter >= 0 && backward_iter != halfedge;
                    backward_iter = get_he_twinID(mesh.halfedges[backward_iter])
                    )
                {
                    ++valence;
                    backward_iter = get_he_nextID(mesh.halfedges[backward_iter]);
                }

                    const float     w = 1.f / (float)valence;
                    const glm::vec3 v = get_vert_point(mesh, vertID);
                    const glm::vec3 f = get_face_point(mesh, faceID);
                    const glm::vec3 e = get_edge_point(mesh, edgeID);
                    const float     s = forward_iter < 0 ? 0.f : 1.f;
                    CAT_CRITICAL
                    { 
                        mesh.vertices[curr_vertID] = mesh.vertices[curr_vertID] + w * (v + w * s * (4.f * e - f - 3.f * v));
                    }                    
            }
        CAT_PARALLEL_BARRIER
    }


    
    /*=====================================================================
    * The reconstruction funcs
    * 
    */


    // helpers:
    std::vector<int32_t> reconstruct_twinID(Cat_Mesh& mesh, const int32_t halfedgeID)
    {
        int32_t t0, t1, t2, t3;
        t0 = 4 * get_nextID_by_heID(mesh, get_twinID_by_heID(mesh, halfedgeID)) + 3;
        t1 = 4 * get_nextID_by_heID(mesh, halfedgeID) + 2;
        t2 = 4 * get_prevID_by_heID(mesh, halfedgeID) + 1;
        t3 = 4 * get_twinID_by_heID(mesh, get_prevID_by_heID(mesh, halfedgeID));

        return { t0,t1,t2,t3 };
    }

    std::vector<int32_t> reconstruct_nextID(const int32_t halfedgeID)
    {
        int32_t n0, n1, n2, n3;
        int32_t fourTimesID = 4 * halfedgeID;
        n0 = fourTimesID + 1;
        n1 = fourTimesID + 2;
        n2 = fourTimesID + 3;
        n3 = fourTimesID;

        return { n0,n1,n2,n3 };
    }

    std::vector<int32_t> reconstruct_prevID(const int32_t halfedgeID)
    {
        int32_t p0, p1, p2, p3;
        int32_t fourTimesID = 4 * halfedgeID;
        p0 = fourTimesID + 3;
        p1 = fourTimesID;
        p2 = fourTimesID + 1;
        p3 = fourTimesID + 2;

        return { p0,p1,p2,p3 };
    }
                                       
    std::vector<int32_t> reconstruct_faceID(const int32_t halfedgeID)
    {
        return { halfedgeID,halfedgeID,halfedgeID,halfedgeID };
    }
                                        
    std::vector<int32_t> reconstruct_edgeID(Cat_Mesh& mesh, const int32_t halfedgeID)
    {
        const int32_t edge_count = mesh.edge_count;
        const int32_t prev_halfedgeID = get_prevID_by_heID(mesh, halfedgeID);

        int32_t e0, e1, e2, e3;
        e0 = (halfedgeID > get_twinID_by_heID(mesh,halfedgeID)) ? 2 * get_edgeID_by_heID(mesh, halfedgeID)
            : 2 * get_edgeID_by_heID(mesh, halfedgeID) + 1;
        e1 = 2 * edge_count + halfedgeID;
        e2 = 2 * edge_count + prev_halfedgeID;
        e3 = (prev_halfedgeID > get_twinID_by_heID(mesh, prev_halfedgeID)) ? 2 * get_edgeID_by_heID(mesh, prev_halfedgeID) + 1 
            : 2 * get_edgeID_by_heID(mesh, prev_halfedgeID);

        return { e0,e1,e2,e3 };
    }
                                      
    std::vector<int32_t> reconstruct_vertID(Cat_Mesh& mesh, const int32_t halfedgeID)
    {
        const int32_t face_count = mesh.face_count;
        const int32_t vert_count = mesh.vertex_count;
        const int32_t prev_halfedgeID = get_prevID_by_heID(mesh, halfedgeID);

        int32_t v0, v1, v2, v3;
        v0 = get_vertID_by_heID(mesh, halfedgeID);
        v1 = get_edgeID_by_heID(mesh, halfedgeID) + vert_count + face_count;
        v2 = get_faceID_by_heID(mesh, halfedgeID) + vert_count;
        v3 = get_edgeID_by_heID(mesh, prev_halfedgeID) + vert_count + face_count;

        return { v0,v1,v2,v3 };
    }


    // delete the old data: 
    //      1) old halfedges 
    //      2) old vertices
    void erase_old_halfedges_and_verts(Cat_Mesh& mesh)
    {
        const int32_t old_halfedge_count = mesh.halfedge_count;
        mesh.halfedges.erase(mesh.halfedges.begin(), mesh.halfedges.begin() + old_halfedge_count);

        const int32_t old_vert_size = mesh.vertex_count;
        mesh.vertices.erase(mesh.vertices.begin(), mesh.vertices.begin() + old_vert_size);
    }

    // generate new halfedges and its relative data
    void reconstruct_mesh_halfedges(Cat_Mesh& mesh)
    {
        int32_t old_halfedge_count = mesh.halfedge_count;
        int32_t new_halfedge_count = old_halfedge_count + mesh.halfedge_count * 4;

        mesh.halfedges.reserve(new_halfedge_count);
        mesh.halfedges.resize(new_halfedge_count);

        CAT_PARALLEL_FOR
            for (int32_t halfedge = 0; halfedge < old_halfedge_count; ++halfedge)
            {
                std::vector<int32_t> twins, nexts, prevs, verts, edges, faces;
                faces = reconstruct_faceID(halfedge);
                nexts = reconstruct_nextID(halfedge);
                prevs = reconstruct_prevID(halfedge);
                verts = reconstruct_vertID(mesh, halfedge);
                edges = reconstruct_edgeID(mesh, halfedge);
                twins = reconstruct_twinID(mesh, halfedge);

                const int32_t newID = 4 * halfedge + old_halfedge_count;
                for (int32_t i = 0; i < 4; ++i)
                    CAT_CRITICAL
                {
                    mesh.halfedges[newID + i].Face = faces[i];
                    mesh.halfedges[newID + i].Next = nexts[i];
                    mesh.halfedges[newID + i].Prev = prevs[i];
                    mesh.halfedges[newID + i].Vert = verts[i];
                    mesh.halfedges[newID + i].Edge = edges[i];
                    mesh.halfedges[newID + i].Twin = twins[i];
                }
            }
        CAT_PARALLEL_BARRIER
    }

    void update_VEFH_counts(Cat_Mesh& mesh)
    {
        mesh.vertex_count += mesh.edge_count + mesh.face_count;

        mesh.edge_count *= 2;
        mesh.edge_count += mesh.halfedge_count;

        mesh.face_count = mesh.halfedge_count;
        mesh.halfedge_count *= 4;       

    }


    void reconstruct_obj(Objmodel& obj, Cat_Mesh& mesh)
    {
        const int32_t new_index_size = mesh.halfedge_count;
        obj.indices.reserve(new_index_size);
        obj.indices.resize(new_index_size);
        std::cout << "new_index_size: " << new_index_size << std::endl;

        //CAT_PARALLEL_FOR
            for (int32_t id = 0; id < new_index_size; ++id)
            {
                obj.indices[id] = get_vertID_by_heID(mesh, id);
            }
        //CAT_PARALLEL_BARRIER

        

        const int32_t new_vert_size = mesh.vertex_count;
        obj.vertices.reserve(new_vert_size);
        obj.vertices.resize(new_vert_size);
        std::cout << "new_vert_size: " << new_vert_size << std::endl;

        //CAT_PARALLEL_FOR
            for (int32_t vert = 0; vert < new_vert_size; ++vert)
            {
                obj.vertices[vert] = mesh.vertices[vert];
            }
        //CAT_PARALLEL_BARRIER
       
    }

    void output_obj(Objmodel& obj, Cat_Mesh& mesh)
    {
        const int32_t new_index_size = mesh.halfedge_count;
        obj.indices.reserve(new_index_size);
        obj.indices.resize(new_index_size);
        //std::cout << "new_index_size: " << new_index_size << std::endl;

        //CAT_PARALLEL_FOR
        for (int32_t id = 0; id < new_index_size; ++id)
        {
            obj.indices[id] = get_vertID_by_heID(mesh, id);
        }
        //CAT_PARALLEL_BARRIER

        const int32_t new_vert_size = mesh.vertex_count;
        obj.vertices.reserve(new_vert_size);
        obj.vertices.resize(new_vert_size);
        //std::cout << "new_vert_size: " << new_vert_size << std::endl;

        //CAT_PARALLEL_FOR
        for (int32_t vert = 0; vert < new_vert_size; ++vert)
        {
            obj.vertices[vert] = mesh.vertices[vert];
        }
        //CAT_PARALLEL_BARRIER
        
    }


    void catmull_parallel(Objmodel& model, int32_t depth)
    {
        Cat_Mesh cat_mesh = obj_2_catmesh(model);
        std::cout << "=========================================="
            << "target mesh has: \n"
            << "vert: " << cat_mesh.vertex_count << "\n"
            << "face: " << cat_mesh.face_count << "\n"
            << "==========================================\n";


        for (int32_t curr_depth = 0; curr_depth < depth; ++curr_depth)
        {
            compute_face_point_parallel(cat_mesh);
            std::cout << " ****** face point generated! ******" << std::endl;
            compute_edge_point_parallel(cat_mesh);
            std::cout << " ****** edge point generated! ******" << std::endl;
            compute_vert_point_parallel(cat_mesh);
            std::cout << " ****** new vert point generated! ******" << std::endl;


            reconstruct_mesh_halfedges(cat_mesh);
            erase_old_halfedges_and_verts(cat_mesh);
            update_VEFH_counts(cat_mesh);
        }     
         //std::cout << "vert count: " << cat_mesh.vertex_count << "\n";
         //std::cout << "edge count: " << cat_mesh.edge_count << "\n" << "face count: " << cat_mesh.face_count << "\n";
         //std::cout << "half count: " << cat_mesh.halfedge_count << "\n";
         


        //reconstruct_obj(model, cat_mesh);
        //output_obj(model, cat_mesh);
    }




    int32_t get_he_faceID(HalfEdge& halfedge)
    {
        return halfedge.Face;
    }

    int32_t get_he_vertID(HalfEdge& halfedge)
    {
        return halfedge.Vert;
    }

    int32_t get_he_nextID(HalfEdge& halfedge)
    {
        return halfedge.Next;
    }

    int32_t get_he_twinID(HalfEdge& halfedge)
    {
        return halfedge.Twin;
    }

    int32_t get_he_prevID(HalfEdge& halfedge)
    {
        return halfedge.Prev;
    }

    int32_t get_he_edgeID(HalfEdge& halfedge)
    {
        return halfedge.Edge;
    }

    glm::vec3 get_vert_by_heID(Cat_Mesh& mesh, int32_t halfedgeID)
    {
        auto vertexID = mesh.halfedges[halfedgeID].Vert;
        assert(vertexID < mesh.vertices.size() && "******** ERR::FUNC: get_vert_by_he , vertexID out of range! ********\n");
        return mesh.vertices[vertexID];
    }

    glm::vec3 get_face_point(Cat_Mesh& mesh, const int32_t faceID)
    {
        const int32_t vertID = mesh.vertex_count + mesh.vertex_count + faceID;
        return mesh.vertices[vertID];
    }

    glm::vec3 get_edge_point(Cat_Mesh& mesh, const int32_t edgeID)
    {
        const int32_t vertID = mesh.vertex_count + mesh.vertex_count + mesh.face_count + edgeID;
        return mesh.vertices[vertID];
    }

    glm::vec3 get_vert_point(Cat_Mesh& mesh, const int32_t vertID)
    {        

        return mesh.vertices[vertID];
    }

    int32_t get_faceID_by_heID(Cat_Mesh& mesh, const int32_t heID)
    {
        assert(heID < mesh.halfedge_count && "******** ERR::FUNC: get_faceID_by_heID , heID out of range! ********\n");
        return get_he_faceID(mesh.halfedges[heID]);
    }

    int32_t get_edgeID_by_heID(Cat_Mesh& mesh, const int32_t heID)
    {
        assert(heID < mesh.halfedge_count && "******** ERR::FUNC: get_edgeID_by_heID , heID out of range! ********\n");
        return get_he_edgeID(mesh.halfedges[heID]);
    }

    int32_t get_vertID_by_heID(Cat_Mesh& mesh, const int32_t heID)
    {
        assert(heID < mesh.halfedge_count && "******** ERR::FUNC: get_vertID_by_heID , heID out of range! ********\n");
        return get_he_vertID(mesh.halfedges[heID]);
    }

    int32_t get_twinID_by_heID(Cat_Mesh& mesh, const int32_t heID)
    {
        assert(heID < mesh.halfedge_count && "******** ERR::FUNC: get_twinID_by_heID , heID out of range! ********\n");
        return get_he_twinID(mesh.halfedges[heID]);
    }

    int32_t get_nextID_by_heID(Cat_Mesh& mesh, const int32_t heID)
    {
        assert(heID < mesh.halfedge_count && "******** ERR::FUNC: get_nextID_by_heID , heID out of range! ********\n");
        return get_he_nextID(mesh.halfedges[heID]);
    }

    int32_t get_prevID_by_heID(Cat_Mesh& mesh, const int32_t heID)
    {
        assert(heID < mesh.halfedge_count && "******** ERR::FUNC: get_prevID_by_heID , heID out of range! ********\n");
        return get_he_prevID(mesh.halfedges[heID]);
    }
}