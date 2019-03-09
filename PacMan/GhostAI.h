#pragma once
#include "Component.h"
#include "Math.h"
#include <vector>
#include <unordered_map>

class GhostAI : public Component
{
public:
	// Used to track the four different GhostAI states
	//At each intersection, they either have to go forward, turn left, or turn right
	enum State
	{
		Scatter, //ghost paths from curr pos to home node. If reaches scatter node, continue circling til state changes
		Chase, //ghost paths to designated target node (typically somehow relative to PacMan’s position)
		Frightened, // ghost turns blue and picks random node to turn to at every intersection
		Dead // ghost turns into eyes and paths back to home pen area, then comes back to life
	};
	
	GhostAI(class Actor* owner);

	void Update(float deltaTime) override;
	void FrightenedUpdate(float deltaTime);
	void ScatterUpdate(float deltaTime);
	void DeadUpdate(float deltaTime);
	void ChaseUpdate(float deltaTime);
	
	// Called when the Ghost starts at the beginning
	// (or when the ghosts should respawn)
	void Start(class PathNode* startNode);

	bool AStar(PathNode* startNode, PathNode* goalNode, PathNode* currentNode);
	
	// Get the current state
	State GetState() const { return mState; }
	
	// Called when the ghost should switch to the "Frightened" state
	void Frighten();
	
	// Called when the ghost should switch to the "Dead" state
	void Die();

	//  Helper function to draw GhostAI's current path
	void DebugDrawPath(struct SDL_Renderer* render);
private:
	// Member data for pathfinding

	float computeG(PathNode* x, PathNode* goal, std::unordered_map<PathNode*, float> mG);
	float computeH(PathNode* x, PathNode* goal, std::unordered_map<PathNode*, float> mH);

	// TargetNode is our current goal node
	class PathNode* mTargetNode = nullptr;
	// PrevNode is the last node we intersected
	// with prior to the current position
	class PathNode* mPrevNode = nullptr;
	// NextNode is the next node we're trying
	// to get to
	class PathNode* mNextNode = nullptr;

	// This vector always contains the path
	// from "next node" to "target node"
	// (if there is still such a path)
	std::vector<class PathNode*> mPath;

	// Current state of the Ghost AI
	State mState = Scatter;

	// Save the owning actor (cast to a Ghost*)
	class Ghost* mGhost;

	// TODO: Add any member data/helper functions here!
	Vector2 mMoveDir = Vector2::Zero;
	Vector2 mFacingDir = Vector2::UnitX;
	Vector2 mInput = Vector2::Zero;
	std::string dir;

	float scatterTime;
	float chaseTime;
};
