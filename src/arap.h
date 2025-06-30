#pragma once

#include "graphics/shape.h"
#include "Eigen/StdList"
#include "Eigen/StdVector"
#include "Eigen/Sparse"

class Shader;

class ARAP
{
private:
    Shape m_shape;
    std::vector<Eigen::Vector3i> m_triangles;
    struct Vertex{
        //list of the indices of adjacent vertices
        Eigen::Vector3f position;
        std::vector<int> adjacent_verts;
        std::unordered_map<int,float> weights;
        //std::vector<std::vector<int>> adj_shared_neigh;
        //WILL NEED TO SET THESE!!!!!!!
        Eigen::MatrixX<float> W;
        Eigen::MatrixX<float> P;
        Eigen::MatrixX<float> P_prime;
        Eigen::MatrixX<float> R;
    };
    std::vector<Vertex> Vertices;
    Eigen::MatrixX<float> L;
    Eigen::SparseMatrix<float> sparseL;
    Eigen::MatrixX<float> b;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> solver;
    void build_L();
    void build_b();
    void build_R(Vertex* vert, std::vector<Eigen::Vector3f> positions, std::vector<Eigen::Vector3f> new_positions);
    void set_b_values(std::vector<Eigen::Vector3f> old_positions, std::vector<Eigen::Vector3f> new_positions, const std::unordered_set<int> anchors);

    void set_Vertices_P(std::vector<Eigen::Vector3f> verts);
    void set_Vertices_P_prime(std::vector<Eigen::Vector3f> verts);

    void delete_anchor_rows(const std::unordered_set<int> anchors);
    void set_L_values();

public:
    ARAP();

    void init(Eigen::Vector3f &min, Eigen::Vector3f &max);
    void move(int vertex, Eigen::Vector3f pos);
    void find_weights();

    // ================== Students, If You Choose To Modify The Code Below, It's On You

    int getClosestVertex(Eigen::Vector3f start, Eigen::Vector3f ray, float threshold)
    {
        return m_shape.getClosestVertex(start, ray, threshold);
    }

    void draw(Shader *shader, GLenum mode)
    {
        m_shape.draw(shader, mode);
    }

    SelectMode select(Shader *shader, int vertex)
    {
        return m_shape.select(shader, vertex);
    }

    bool selectWithSpecifiedMode(Shader *shader, int vertex, SelectMode mode)
    {
        return m_shape.selectWithSpecifiedMode(shader, vertex, mode);
    }

    bool getAnchorPos(int lastSelected, Eigen::Vector3f& pos, Eigen::Vector3f ray, Eigen::Vector3f start)
    {
        return m_shape.getAnchorPos(lastSelected, pos, ray, start);
    }
};
