#include "Morai_Woowa/AStar.hpp"
#include <algorithm>
#include <math.h>
#include <iostream>

using namespace std::placeholders;
 
bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

float AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setBackMovement(false);
    setHeuristic(&Heuristic::manhattan);
}

void AStar::Generator::setWorldSize(Vec2i worldSize_x_, Vec2i worldSize_y_)
{
    worldSize_x = worldSize_x_;
    worldSize_y = worldSize_y_;
}

// void AStar::Generator::setWorldSize(Vec2i worldSize_)
// {
//     worldSize = worldSize_;
// }

void AStar::Generator::setGridSize(float GridSize_)
{
    GridSize = GridSize_;
        
    direction = {
        { 0, GridSize }, { GridSize, 0 }, { 0, -GridSize }, { GridSize, GridSize },
        { GridSize, -GridSize }, { -GridSize, 0 }, { -GridSize, GridSize }, { -GridSize, -GridSize }
    };
}

void AStar::Generator::setBackMovement(bool enable_)
{
    directions = (enable_ ? 8 : 5);
}

void AStar::Generator::setCollisionDis(float dis)
{
    collision_dis = dis;
}

void AStar::Generator::setCollisionDis_main(float dis)
{
    collision_dis_main = dis;
}

void AStar::Generator::setCollisionDis_sub(float dis)
{
    collision_dis_sub = dis;
}


void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(const std::vector<std::vector<float>>& point_vector) 
{
    // point_vector의 모든 요소를 walls에 추가합니다.
    for (const auto& point : point_vector) {
        walls.push_back({point[0], point[1]});
    }
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    openSet.reserve(1000);
    closedSet.reserve(1000);
    openSet.push_back(new Node(source_));
    std::cout << collision_dis << ": collision_dis" << std::endl;

    while (!openSet.empty()) {
    //for(int i=0; i<10000;i++){
        auto current_it = openSet.begin();
        current = *current_it;
        // .begin() > 첫 번째 idx
        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) 
            {
                current = node;
                current_it = it;
            }
        }

        // std::cout << current->coordinates.x  << std::endl; 
        // std::cout << current->coordinates.y  << std::endl; 
        // std::cout << current->G  << std::endl; 
        // std::cout << current->H  << std::endl; 
        // std::cout << i << std::endl; 

        float dis = heuristic(current->coordinates, target_);
        
        if (dis < 0.1) {
            //std::cout << "closedSet.size() : " << closedSet.size() << std::endl;
            printf("끝!!!!!!\n");
            //setCollisionDis(collision_dis_main);
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        for (uint i = 0; i < directions; ++i) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates, source_) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            float totalCost = current->G + ((i < 4) ? GridSize : sqrt(GridSize*2));

            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
        // std::cout << "closedSet.size() : " << closedSet.size() << std::endl;
        // std::cout << "openSet.size() : " << openSet.empty() << std::endl;
        // std::cout << current->coordinates.x << "," << current->coordinates.y << std::endl;
        
        if(openSet.empty())
        {
            setCollisionDis(collision_dis_sub);
            //Node *current = nullptr;
            break;
        }
    }

    CoordinateList path;
    
    while (current != nullptr) 
    {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::decision_collision(Vec2i coordinates_)
{
    for(auto obs : walls){
        float dis = heuristic(obs, coordinates_);
        if(dis < collision_dis)
            return true;   
    }
    return false;
}

bool AStar::Generator::detectCollision(Vec2i coordinates_, Vec2i source_)
{
    float dis_from_source = heuristic(source_, coordinates_);

    //std::cout << dis_from_source << "dis_from_source" << std::endl;

    // if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
    //     coordinates_.y < -worldSize.y/2 || coordinates_.y >= worldSize.y/2 ||

    if (coordinates_.x < worldSize_x.x || coordinates_.x >= worldSize_x.y ||
        coordinates_.y < worldSize_y.x || coordinates_.y >= worldSize_y.y) 
    {
        return true;
    }
    
    if (dis_from_source < 1.4 && decision_collision(coordinates_))
    {
        return true;
    }
    
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

float AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    float distance = sqrt(pow(delta.x, 2) + pow(delta.y, 2));
    //std::cout << "distance" << distance << std::endl;
    return distance;
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
