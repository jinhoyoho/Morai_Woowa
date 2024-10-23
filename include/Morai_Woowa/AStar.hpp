/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>


namespace AStar
{
    struct Vec2i
    {
        float x, y;

        bool operator == (const Vec2i& coordinates_);
        friend Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_) {
            return{ left_.x + right_.x, left_.y + right_.y };
        }
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<float(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;

    struct Node
    {
        float G, H;
        Vec2i coordinates;
        Node *parent;

        Node(Vec2i coord_, Node *parent_ = nullptr);
        float getScore();
    };
    
    using NodeSet = std::vector<Node*>;

    class Generator
    {
        bool detectCollision(Vec2i coordinates_, Vec2i source_);
        //bool detectCollision(Vec2i coordinates_);

        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);
        void releaseNodes(NodeSet& nodes_);
        bool decision_collision(Vec2i coordinates_);

    public:
        Generator();
        void setWorldSize(Vec2i worldSize_x_, Vec2i worldSize_y_);
        //void setWorldSize(Vec2i worldSize);

        void setBackMovement(bool enable_);
        //void decision_collision(Vec2i coordinates_);

        void setCollisionDis(float dis);
        void setCollisionDis_main(float dis_main);
        void setCollisionDis_sub(float dis_sub);

        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList findPath(Vec2i source_, Vec2i target_);
        void addCollision(const std::vector<std::pair<float, float>>& point_vector);
        void removeCollision(Vec2i coordinates_);
        void clearCollisions();
        void setGridSize(float GridSize_);

    private:
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        Vec2i worldSize_x;
        Vec2i worldSize_y;
        uint directions;
        float GridSize = 1;
        float collision_dis;
        float collision_dis_main;
        float collision_dis_sub;

    };

    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static uint manhattan(Vec2i source_, Vec2i target_);
        static float euclidean(Vec2i source_, Vec2i target_);
        static uint octagonal(Vec2i source_, Vec2i target_);
    };
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
