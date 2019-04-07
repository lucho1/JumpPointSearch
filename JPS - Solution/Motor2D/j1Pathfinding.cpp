#include "p2Defs.h"
#include "p2Log.h"
#include "j1App.h"
#include "j1PathFinding.h"


j1PathFinding::j1PathFinding() : j1Module(), map(NULL), width(0), height(0)
{
	name.assign("pathfinding");
}

// Destructor
j1PathFinding::~j1PathFinding()
{
	RELEASE_ARRAY(map);
}

// Called before quitting
bool j1PathFinding::CleanUp()
{
	LOG("Freeing pathfinding library");

	last_path.clear();
	RELEASE_ARRAY(map);
	return true;
}

// Sets up the walkability map
void j1PathFinding::SetMap(uint width, uint height, uchar* data)
{
	this->width = width;
	this->height = height;

	RELEASE_ARRAY(map);
	map = new uchar[width*height];
	memcpy(map, data, width*height);
}

// Utility: return true if pos is inside the map boundaries
bool j1PathFinding::CheckBoundaries(const iPoint& pos) const
{
	return (pos.x >= 0 && pos.x <= (int)width &&
			pos.y >= 0 && pos.y <= (int)height);
}

// Utility: returns true is the tile is walkable
bool j1PathFinding::IsWalkable(const iPoint& pos) const
{
	uchar t = GetTileAt(pos);
	return t != INVALID_WALK_CODE && t > 0;
}

// Utility: return the walkability value of a tile
uchar j1PathFinding::GetTileAt(const iPoint& pos) const
{
	if(CheckBoundaries(pos))
		return map[(pos.y*width) + pos.x];

	return INVALID_WALK_CODE;
}

// To request all tiles involved in the last generated path
const std::vector<iPoint>* j1PathFinding::GetLastPath() const
{
	return &last_path;
}

// PathList ------------------------------------------------------------------------
// Looks for a node in this list and returns it's list node or list.end()
// ---------------------------------------------------------------------------------
std::list<PathNode>::iterator PathList::Find(const iPoint& point)
{
	std::list<PathNode>::iterator item = list.begin();

	while(item != list.end())
	{
	
		if ((*item).pos == point)
			return item;

		item = next(item);
	}

	return list.end();
}

// PathList ------------------------------------------------------------------------
// Returns the Pathnode with lowest score in this list or NULL if empty
// ---------------------------------------------------------------------------------
std::list<PathNode>::iterator PathList::GetNodeLowestScore()
{
	std::list<PathNode>::iterator ret;
	int min = 65535;

	std::list<PathNode>::iterator item = list.begin();

	while(item != list.end())
	{
		if((*item).Score() < min)
		{
			min = (*item).Score();
			ret = item;
		}
		item = next(item);
	}
	return ret;
}

// PathNode -------------------------------------------------------------------------
// Convenient constructors
// ----------------------------------------------------------------------------------
PathNode::PathNode() : g(-1), h(-1), pos(-1, -1), parent(NULL)
{}

PathNode::PathNode(int g, int h, const iPoint& pos, PathNode* parent) : g(g), h(h), pos(pos), parent(parent)
{}

PathNode::PathNode(const PathNode& node) : g(node.g), h(node.h), pos(node.pos), parent(node.parent)
{}

// PathNode -------------------------------------------------------------------------
// Fills a list (PathList) of all valid adjacent pathnodes
// ----------------------------------------------------------------------------------
uint PathNode::FindWalkableAdjacents(PathList& list_to_fill)
{
	iPoint cell;

	// north
	cell.create(pos.x, pos.y + 1);
	if(App->pathfinding->IsWalkable(cell))
		list_to_fill.list.push_back(PathNode(-1, -1, cell, this));

	// south
	cell.create(pos.x, pos.y - 1);
	if(App->pathfinding->IsWalkable(cell))
		list_to_fill.list.push_back(PathNode(-1, -1, cell, this));

	// east
	cell.create(pos.x + 1, pos.y);
	if(App->pathfinding->IsWalkable(cell))
		list_to_fill.list.push_back(PathNode(-1, -1, cell, this));

	// west
	cell.create(pos.x - 1, pos.y);
	if(App->pathfinding->IsWalkable(cell))
		list_to_fill.list.push_back(PathNode(-1, -1, cell, this));

	//DIAGONAL
	// north-east
	cell.create(pos.x + 1, pos.y + 1);
	if (App->pathfinding->IsWalkable(cell))
		list_to_fill.list.push_back(PathNode(-1, -1, cell, this));

	// south-east
	cell.create(pos.x + 1, pos.y - 1);
	if (App->pathfinding->IsWalkable(cell))
		list_to_fill.list.push_back(PathNode(-1, -1, cell, this));

	//north-west
	cell.create(pos.x - 1, pos.y + 1);
	if (App->pathfinding->IsWalkable(cell))
		list_to_fill.list.push_back(PathNode(-1, -1, cell, this));

	//south-west
	cell.create(pos.x - 1, pos.y - 1);
	if (App->pathfinding->IsWalkable(cell))
		list_to_fill.list.push_back(PathNode(-1, -1, cell, this));

	return list_to_fill.list.size();
}

// PathNode -------------------------------------------------------------------------
// Calculates this tile score
// ----------------------------------------------------------------------------------
int PathNode::Score() const
{
	return g + h;
}

// PathNode -------------------------------------------------------------------------
// Calculate the F for a specific destination tile
// ----------------------------------------------------------------------------------
int PathNode::CalculateF(const iPoint& destination)
{
	g = parent->g + 1;
	h = pos.DistanceTo(destination);

	//You can also try:
	//h = pos.DistanceManhattan(destination);
	//h = pos.DiagonalDistance(destination);
	//h = pos.DistanceNoSqrt(destination);

	return g + h;
}

// ----------------------------------------------------------------------------------
// Actual A* algorithm: return number of steps in the creation of the path or -1 ----
// ----------------------------------------------------------------------------------
int j1PathFinding::PropagateAStar(const iPoint& origin, const iPoint& destination) {

	int ret = -1; 

	PathList open;
	PathList close;

	open.list.push_back(PathNode(0, 0, origin, NULL));

	while (open.list.size() > 0) {

		std::list<PathNode>::iterator aux = open.GetNodeLowestScore();
		close.list.push_back(*aux);

		std::list<PathNode>::iterator lower = prev(close.list.end());
		open.list.erase(aux);

		if ((*lower).pos == destination) {

			last_path.clear();
			const PathNode *new_node = &(*lower);

			while (new_node) {

				last_path.push_back(new_node->pos);
				new_node = new_node->parent;
			}

			std::reverse(last_path.begin(), last_path.end());
			ret = last_path.size();
			break;
		}

		PathList AdjacentNodes;
		AdjacentNodes.list.clear();

		(*lower).FindWalkableAdjacents(AdjacentNodes);
		std::list<PathNode>::iterator it = AdjacentNodes.list.begin();

		for (; it != AdjacentNodes.list.end(); it = next(it)) {

			if (close.Find((*it).pos) != close.list.end())
				continue;

			std::list<PathNode>::iterator adj_node = open.Find((*it).pos);

			if (adj_node == open.list.end()) {

				(*it).CalculateF(destination);
				open.list.push_back(*it);
			}
			else if ((*adj_node).g > (*it).g + 1) {

				(*adj_node).parent = (*it).parent;
				(*adj_node).CalculateF(destination);

			}
		}
	}

	return ret;
}


int j1PathFinding::CreatePath(const iPoint& origin, const iPoint& destination, bool JPS_active) {

	int ret = -1;

	if (!IsWalkable(origin) || !IsWalkable(destination))
		return ret;

	if (JPS_active == false)
		PropagateAStar(origin, destination);
	else
		PropagateJPS(origin, destination);

	LOG("Path Steps: %i", last_path.size());

	return ret;
}


// ----------------------------------------------------------------------------------
// JPS algorithm: returns number of steps in the creation of the path or -1 ----
// ----------------------------------------------------------------------------------
int j1PathFinding::PropagateJPS(const iPoint& origin, const iPoint& destination) {

	int ret = -1;

	PathList open;
	PathList close;

	open.list.push_back(PathNode(0, 0, origin, NULL));

	while (open.list.size() > 0) {

		std::list<PathNode>::iterator aux = open.GetNodeLowestScore();
		close.list.push_back(*aux);

		std::list<PathNode>::iterator lower = prev(close.list.end());
		open.list.erase(aux);

		if ((*lower).pos == destination) {

			last_path.clear();
			const PathNode* new_node = &(*lower);

			while (new_node) {

				last_path.push_back(new_node->pos);
				new_node = new_node->parent;
			}

			std::reverse(last_path.begin(), last_path.end());
			ret = last_path.size();
			break;
		}

		PathList AdjacentNodes = lower->PruneNeighbours(destination, this); //THIS IS THE ONLY DIFFERENCE WITH A*'s core

		std::list<PathNode>::iterator it = AdjacentNodes.list.begin();
		for (; it != AdjacentNodes.list.end(); it = next(it)) {

			if (close.Find((*it).pos) != close.list.end())
				continue;

			std::list<PathNode>::iterator adj_node = open.Find((*it).pos);

			if (adj_node == open.list.end()) {

				(*it).CalculateF(destination);
				open.list.push_back(*it);
			}
			else if ((*adj_node).g > (*it).g + 1) {

				(*adj_node).parent = (*it).parent;
				(*adj_node).CalculateF(destination);

			}
		}
	}

	return ret;
}


PathList PathNode::PruneNeighbours(const iPoint& destination, j1PathFinding* PF_Module) {
	
	PathList ret;

	//Fill the neighbours list with the whole tile neighbours
	PathList neighbours;
	FindWalkableAdjacents(neighbours); 

	//Now we will iterate it and select ONLY the neighbours that we want
	std::list<PathNode>::iterator item = neighbours.list.begin();
	for (; item != neighbours.list.end(); item = next(item)) {

		//Direction from current node to the node which we are iterating
		//Clamp is to make sure that direction is kept inside unitary factors (if d < -1, return -1 || if d > 1, return 1. Otherwise return d)
		iPoint direction(CLAMP((*item).pos.x - pos.x, 1, -1), CLAMP((*item).pos.y - pos.y, 1, -1));

		//Now Find a Jump Point and add it to the list if found
		PathNode* JumpPoint = PF_Module->Jump(pos, direction, destination, this);
		if (JumpPoint != nullptr)
			ret.list.push_back(*JumpPoint);
	}

	return ret;
}


PathNode* j1PathFinding::Jump(iPoint current_position, iPoint direction, const iPoint& destination, PathNode* parent) {

	iPoint next(current_position.x + direction.x, current_position.y + direction.y);

	if (IsWalkable(next) == false) //If next point isn't walkable, return nullptr
		return nullptr;

	PathNode *retJP = new PathNode(-1, -1, next, parent);

	if (retJP->pos == destination) //If next node is goal, return it
		return retJP;

	//Check diagonal directions
	if (direction.x != 0 && direction.y != 0) {

		if (IsWalkable(current_position + iPoint(direction.x, 0)) == false)
			return retJP;
		else if (IsWalkable(current_position + iPoint(0, direction.y)) == false)
			return retJP;

		if (Jump(next, iPoint(direction.x, 0), destination, parent) != nullptr
			|| Jump(next, iPoint(0, direction.y), destination, parent) != nullptr)
			return retJP;
	}
	else { //Check Straight Directions

		if (direction.x != 0) { //Horizontal

			if (IsWalkable(current_position + iPoint(0, 1)) == false) {

				if (IsWalkable(current_position + iPoint(direction.x, 1)) == true)
					return retJP;
			}
			else if (IsWalkable(current_position + iPoint(0, -1)) == false)
				if (IsWalkable(current_position + iPoint(direction.x, -1)) == true)
					return retJP;
		}
		else { //Vertical

			if (IsWalkable(current_position + iPoint(1, 0)) == false) {

				if (IsWalkable(current_position + iPoint(1, direction.y)) == true)
					return retJP;
			}
			else if (IsWalkable(current_position + iPoint(-1, 0)) == false)
				if (IsWalkable(current_position + iPoint(-1, direction.y)) == true)
					return retJP;
		}
	}

	return Jump(next, direction, destination, parent);
}