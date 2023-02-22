#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
  start_node = &(m_Model.FindClosestNode(start_x, start_y));
  end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

// TODO 3: Implement the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  // Get distance to the end_node for the h value.
  // Node objects have a distance method to determine the distance to another node.
  return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  // The FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
  current_node->FindNeighbors();

  // For each node in current_node.neighbors, set the parent, the h_value, the g_value.
  // Note: We can use "auto" instead of RouteModel::Node
  for (auto *neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    // Use CalculateHValue below to implement the h-Value calculation.
    neighbor->h_value = CalculateHValue(neighbor);
    neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
    // For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited
    // attribute to true.
    open_list.push_back(neighbor);
    neighbor->visited = true;
  }
}


RouteModel::Node *RoutePlanner::NextNode() {
  // Sort the open_list according to the sum of the h value and g value.
  // Use lambda - found at
  // C++17 Standard Library Quick Reference (book)
  // https://en.cppreference.com/w/cpp/language/lambda
  // https://www.newthinktank.com/2018/04/c-tutorial-9-2/
  // and other sites
  std::sort(open_list.begin(), open_list.end(), [](const auto &x, const auto &y) {
    return (x->g_value + x->h_value) < (y->g_value + y->h_value);
  });
  // Create a pointer to the node in the list with the lowest sum.
  auto node = open_list.front();
  auto lowest = open_list.begin();
  // Remove that node from the open_list.
  open_list.erase(lowest);

  // Return the pointer.
  return node;
}


// Method takes the current (final) node as an argument and iteratively follow the
// chain of parents of nodes until the starting node is found.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  // For each node in the chain, add the distance from the node to its parent to the distance variable.
  // The returned vector should be in the correct order: the start node should be the first element
  //   of the vector, the end node should be the last element.
  while (current_node->parent != nullptr) {
    path_found.push_back(*current_node);
    distance += current_node->distance(*current_node->parent);
    current_node = current_node->parent;
  }
  path_found.push_back(*current_node);
  std::reverse(path_found.begin(), path_found.end());

  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
  return path_found;
}


void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;

  start_node->visited = true;
  open_list.push_back(start_node);
  while (!open_list.empty()) {
    // The NextNode() method sorts the open_list and return the next node.
    current_node = NextNode();
    // When the search has reached the end_node, use the ConstructFinalPath method to return the final path
    //   that was found.
    // Store the final path in the m_Model.path attribute before the method exits. This path will then be
    //   displayed on the map tile.
    // Note: We can simply check current vs end_node instead of current_node->distance(*end_node) == 0
    if ((current_node->x == end_node->x) && (current_node->y == end_node->y)) {
      m_Model.path = ConstructFinalPath(current_node);
      break;
    }
    // The AddNeighbors method to add all of the neighbors of the current node to the open_list.
    AddNeighbors(current_node);
  }
}