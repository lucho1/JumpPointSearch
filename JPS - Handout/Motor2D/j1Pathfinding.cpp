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

	//TODO 6: Remember to uncomment lines below to be able to go diagonally
	//DIAGONAL
	// north-east
	//cell.create(pos.x + 1, pos.y + 1);
	//if (App->pathfinding->IsWalkable(cell))
	//	list_to_fill.list.push_back(PathNode(-1, -1, cell, this));

	//// south-east
	//cell.create(pos.x + 1, pos.y - 1);
	//if (App->pathfinding->IsWalkable(cell))
	//	list_to_fill.list.push_back(PathNode(-1, -1, cell, this));

	////north-west
	//cell.create(pos.x - 1, pos.y + 1);
	//if (App->pathfinding->IsWalkable(cell))
	//	list_to_fill.list.push_back(PathNode(-1, -1, cell, this));

	////south-west
	//cell.create(pos.x - 1, pos.y - 1);
	//if (App->pathfinding->IsWalkable(cell))
	//	list_to_fill.list.push_back(PathNode(-1, -1, cell, this));

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


		//TODO 1: Only difference with A* in the core behaviour: Instead of filling
		//the Adjacent nodes list with the immediate neighbours, we call the function
		//that must prune them
		PathList AdjacentNodes;

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

	//TODO 2: Here we do the step that A* does in its core and that we deleted in TODO1,
	//Fill the neighbours list with the real or immediate neighbours
	//Then iterate it

	//TODO 3: For each iteration, calculate the direction from current node
	//to its neighbour. You can use CLAMP (defined in p2Defs)

	//TODO 4: Make a Jump towards the calculated direction to find the next Jump Point
	//and, if any Jump Point is found, add it to the list that we must return
	

	return ret;
}


PathNode* j1PathFinding::Jump(iPoint current_position, iPoint direction, const iPoint& destination, PathNode* parent) {

	//Determine next possible Jump Point's Position
	iPoint JumpPoint_pos(current_position.x + direction.x, current_position.y + direction.y);

	//If next point isn't walkable, return nullptr
	if (IsWalkable(JumpPoint_pos) == false)
		return nullptr;

	PathNode *ret_JumpPoint = new PathNode(-1, -1, JumpPoint_pos, parent);

	//If next point is goal, return it
	if (ret_JumpPoint->pos == destination)
		return ret_JumpPoint;


	//TODO 5: Check if there is any possible Jump Point in Straight Directions (horizontal and vertical)
	//If found any, return it. Else, keep jumping in the same direction
	/// Checking Horizontals

	//TODO 6: Do the same check than for Straight Lines but now for Diagonals!
	//(Remember prunning rules for diagonals!)
	/// Checking Diagonals


	return Jump(JumpPoint_pos, direction, destination, parent);
}