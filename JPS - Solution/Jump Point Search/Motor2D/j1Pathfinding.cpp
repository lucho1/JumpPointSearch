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

PathNode::PathNode(int g, int h, const iPoint& pos, const PathNode* parent) : g(g), h(h), pos(pos), parent(parent)
{}

PathNode::PathNode(const PathNode& node) : g(node.g), h(node.h), pos(node.pos), parent(node.parent)
{}

// PathNode -------------------------------------------------------------------------
// Fills a list (PathList) of all valid adjacent pathnodes
// ----------------------------------------------------------------------------------
uint PathNode::FindWalkableAdjacents(PathList& list_to_fill) const
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
	//// north-east
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
	h = pos.DistanceManhattan(destination); //For diagonal use DiagonalDistance

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
				LOG("STEPS: %i", last_path.size());
			}

			std::reverse(last_path.begin(), last_path.end());
			ret = last_path.size();
			break;
		}

		PathList AdjacentNodes;
		AdjacentNodes.list.clear();

		(*lower).FindWalkableAdjacents(AdjacentNodes);
		std::list<PathNode>::iterator it = AdjacentNodes.list.begin(); //iterator of Adjacent Nodes list

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


// ----------------------------------------------------------------------------------
// JPS algorithm: returns number of steps in the creation of the path or -1 ----
// ----------------------------------------------------------------------------------
int j1PathFinding::PropagateJPS(const iPoint& origin, const iPoint& destination) {

	int ret = -1;


	return ret;
}

int j1PathFinding::CreatePath(const iPoint& origin, const iPoint& destination) {

	int ret = -1;

	if (!IsWalkable(origin) || !IsWalkable(destination))
		return ret;

	PropagateAStar(origin, destination);

	return ret;
}