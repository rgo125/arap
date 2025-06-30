#include "arap.h"
#include "graphics/meshloader.h"

#include <iostream>
#include <set>
#include <map>
#include <vector>

using namespace std;
using namespace Eigen;

ARAP::ARAP() {}

void ARAP::init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax)
{
    vector<Vector3f> vertices;
    vector<Vector3i> triangles;

    // If this doesn't work for you, remember to change your working directory
    if (MeshLoader::loadTriMesh("/Users/rainergardner-olesen/Desktop/Spring_2025_Courses/Graphics/projects/arap-rgo125/meshes/teapot.obj", vertices, triangles)) {
        m_shape.init(vertices, triangles);
    }

    m_triangles = triangles;

    for(int i = 0 ; i < vertices.size(); i++){
        Vertex vert;
        vert.position = vertices.at(i);
        Vertices.push_back(vert);
    }

    for(int i = 0; i < triangles.size(); i++){
        Vector3i tri = triangles.at(i);

        for(int j = 0; j < 3; j++){
            int neigh_1 = (j+1) % 3;
            int neigh_2 = (j+2) % 3;

            bool can_put_1 = true;
            bool can_put_2 = true;

            for(int k = 0; k < Vertices[tri[j]].adjacent_verts.size(); k++){
                if(tri[neigh_1] == Vertices[tri[j]].adjacent_verts[k]){
                    can_put_1 = false;
                }
            }

            if(can_put_1){
                Vertices[tri[j]].adjacent_verts.push_back(tri[neigh_1]);
                Vertices[tri[j]].weights[tri[neigh_1]] = 0.f;
            }

            for(int k = 0; k < Vertices[tri[j]].adjacent_verts.size(); k++){
                if(tri[neigh_2] == Vertices[tri[j]].adjacent_verts[k]){
                    can_put_2 = false;
                }
            }

            if(can_put_2){
                Vertices[tri[j]].adjacent_verts.push_back(tri[neigh_2]);
                Vertices[tri[j]].weights[tri[neigh_2]] = 0.f;
            }
        }
    }

    for(int i = 0 ; i < Vertices.size(); i++){
        Vertex *v1 = &Vertices[i];

        v1->P.resize(3, v1->adjacent_verts.size());
        v1->P.setZero();

        v1->P_prime.resize(3, v1->adjacent_verts.size());
        v1->P_prime.setZero();

        v1->W.resize(v1->adjacent_verts.size(), v1->adjacent_verts.size());
        v1->W.setZero();

    }


    build_L();
    build_b();


    // Students, please don't touch this code: get min and max for viewport stuff
    MatrixX3f all_vertices = MatrixX3f(vertices.size(), 3);
    int i = 0;
    for (unsigned long i = 0; i < vertices.size(); ++i) {
        all_vertices.row(i) = vertices[i];
    }
    coeffMin = all_vertices.colwise().minCoeff();
    coeffMax = all_vertices.colwise().maxCoeff();
}

void ARAP::find_weights(){

    for(int i = 0; i < Vertices.size(); i++){
        for(auto& pair: Vertices[i].weights){
            pair.second = 0.f;
        }
    }

    for(int i = 0; i < m_triangles.size(); i++){
        Vector3i tri = m_triangles[i];
        for(int j = 0; j < 3; j++){
            int vert_ind = tri[j];
            int neigh_1_ind = tri[(j+1) % 3];
            int neigh_2_ind = tri[(j+2) % 3];
            //REMEMBER TO INITIALIZE ALL WEIGHTS TO 0 AT THE BEGINNING OF THE PROGRAM
            //ALSO NEED TO UPDATE AND KEEP TRACK OF POSITION VALUES
            Vector3f pos_1 = Vertices[vert_ind].position;
            Vector3f pos_2 = Vertices[neigh_1_ind].position;
            Vector3f pos_3 = Vertices[neigh_2_ind].position;


            Vector3f three_to_1 = pos_1 - pos_3;
            Vector3f three_to_2 = pos_2 - pos_3;
            float dot_alpha = three_to_1.dot(three_to_2);
            float cross_alpha = three_to_1.cross(three_to_2).norm();
            float weight = abs(dot_alpha/cross_alpha);

            Vertices[vert_ind].weights[neigh_1_ind] += ((1.f/2.f) * weight);

            Vector3f two_to_1 = pos_1 - pos_2;
            Vector3f two_to_3 = pos_3 - pos_2;
            dot_alpha = two_to_1.dot(two_to_3);
            cross_alpha = two_to_1.cross(two_to_3).norm();
            weight = abs(dot_alpha/cross_alpha);

            Vertices[vert_ind].weights[neigh_2_ind] += ((1.f/2.f) * weight);
        }
    }

    for(int i = 0; i < Vertices.size(); i++){
        for(int j = 0; j < Vertices[i].adjacent_verts.size(); j++){
            Vertices[i].W(j,j) = Vertices[i].weights[Vertices[i].adjacent_verts[j]];
        }
    }
    set_L_values();
}

void ARAP::build_L(){

    L.resize(Vertices.size(), Vertices.size());
    L.setZero();
    std::cout << "L non-zeroes " << (L.array() != 0).count() << std::endl;
}

void ARAP::build_b(){
    b.resize(Vertices.size(), 3);
    b.setZero();
}

void ARAP::build_R(Vertex* vert, std::vector<Vector3f> positions, std::vector<Vector3f> new_positions){
    MatrixXf S = vert->P * vert->W * vert->P_prime.transpose();

    JacobiSVD<MatrixXf> svd(S, ComputeFullV | ComputeFullU );
    Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
    if(R.determinant() < 0){
        Matrix3f newU = svd.matrixU();
        newU.col(2) *= -1;
        R = svd.matrixV() * newU.transpose();
    }
    vert->R = R;
}

void ARAP::delete_anchor_rows(const std::unordered_set<int> anchors){
    for(int a: anchors){
        L.row(a).setZero();
        L.col(a).setZero();
        L(a,a) = 1.f;
    }
    sparseL = L.sparseView();
    sparseL.makeCompressed();
    solver.analyzePattern(sparseL);

    solver.factorize(sparseL);
}

void ARAP::set_L_values(){
    std::cout<<"SET L"<<std::endl;
    std::cout<<"L size" << L.rows()<<"x"<<L.cols()<<std::endl;
    std::cout << "L non-zeroes " << (L.array() != 0).count() << std::endl;

    std::vector<Eigen::Vector3f> verts = m_shape.getVertices();
    const std::unordered_set<int>& anchors = m_shape.getAnchors();

    for(int i = 0; i < verts.size(); i++){

        Vertex vert_1 = Vertices[i];
        float summed_weights = 0.f;
        for(int j = 0; j < vert_1.adjacent_verts.size(); j++){
            L(i, vert_1.adjacent_verts[j]) = -1.f * vert_1.weights[vert_1.adjacent_verts[j]];

            summed_weights += vert_1.weights[vert_1.adjacent_verts[j]];
        }
        L(i,i) = summed_weights;
    }
    sparseL = L.sparseView();
    sparseL.makeCompressed();

    solver.analyzePattern(sparseL);

    solver.factorize(sparseL);
}

void ARAP::set_Vertices_P(std::vector<Eigen::Vector3f> verts){
    for(int i = 0; i < Vertices.size(); i++){
        Vertices[i].position = verts[i];
    }
    for(int i = 0; i < Vertices.size(); i++){
        for(int j = 0; j < Vertices[i].adjacent_verts.size(); j++){
            Vertices[i].P.col(j) = Vertices[i].position - Vertices[Vertices[i].adjacent_verts[j]].position;
        }
    }
}

void ARAP::set_Vertices_P_prime(std::vector<Eigen::Vector3f> verts){
    for(int i = 0; i < Vertices.size(); i++){
        for(int j = 0; j < Vertices[i].adjacent_verts.size(); j++){
            Vertices[i].P_prime.col(j) = verts[i] - verts[Vertices[i].adjacent_verts[j]];
        }
    }
}

void ARAP::set_b_values(std::vector<Vector3f> old_positions, std::vector<Eigen::Vector3f> new_positions, const std::unordered_set<int> anchors){

    for(int i = 0; i < Vertices.size(); i++){
        Vertex vert = Vertices[i];
        Vector3f row(0.f,0.f,0.f);
        for(int j = 0; j < vert.adjacent_verts.size(); j++){

            Vertex neigh_vert = Vertices[vert.adjacent_verts[j]];
            Vector3f value = (vert.W(j,j)/2.f) * ((vert.R + neigh_vert.R) * (old_positions[i] - old_positions[vert.adjacent_verts[j]]));

            row += value;

            if(anchors.count(vert.adjacent_verts[j])){
                row+=(vert.W(j,j) * new_positions[vert.adjacent_verts[j]]);
            }
        }
        b.row(i) = row;
    }

    for(int a: anchors){
        b.row(a) = new_positions[a];
    }

}

// Move an anchored vertex, defined by its index, to targetPosition
void ARAP::move(int vertex, Vector3f targetPosition)
{
    std::vector<Eigen::Vector3f> new_vertices = m_shape.getVertices();
    const std::unordered_set<int>& anchors = m_shape.getAnchors();


    //set_L_values();
    std::vector<Eigen::Vector3f> old_vertices = new_vertices;

    //get rid of the rows and columns for the anchor points
    delete_anchor_rows(anchors);
    // TODO: implement ARAP here
    new_vertices[vertex] = targetPosition;
    //SET THE PS
    set_Vertices_P(old_vertices);
    //SET THE P_PRIMES
    set_Vertices_P_prime(new_vertices);

    for(int j = 0; j < 15; j++){

        for(int i = 0; i < Vertices.size(); i++){
            build_R(&Vertices[i], old_vertices, new_vertices);
        }

        set_b_values(old_vertices, new_vertices, anchors);
        MatrixXf solved = solver.solve(b);

        for(int i = 0; i < new_vertices.size(); i++){
            new_vertices[i] = solved.row(i);
        }
        set_Vertices_P_prime(new_vertices);
    }
    set_Vertices_P(new_vertices);


    // Here are some helpful controls for the application
    //
    // - You start in first-person camera mode
    //   - WASD to move, left-click and drag to rotate
    //   - R and F to move vertically up and down
    //
    // - C to change to orbit camera mode
    //
    // - Right-click (and, optionally, drag) to anchor/unanchor points
    //   - Left-click an anchored point to move it around
    //
    // - Minus and equal keys (click repeatedly) to change the size of the vertices

    m_shape.setVertices(new_vertices);
}
