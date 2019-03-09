#include "GhostAI.h"
#include "Actor.h"
#include "Game.h"
#include "CollisionComponent.h"
#include "Game.h"
#include "PathNode.h"
#include "AnimatedSprite.h"
#include <SDL/SDL.h>
#include <unordered_map>
#include "Ghost.h"
#include "PacMan.h"
#include "Random.h"
#include "Math.h"
#include <iterator>


GhostAI::GhostAI(class Actor* owner)
:Component(owner, 50)
{
	mGhost = static_cast<Ghost*>(owner);
	
	dir = "up";
	scatterTime = 0.0f;
	chaseTime = 20.0f;
}

void GhostAI::FrightenedUpdate(float deltaTime) {
	// Update position:
	if (mGhost->IsDead()) {
		return;
	}

	if (mGhost->frightenedTime >= 0.0f) {
		mGhost->frightenedTime -= deltaTime; // Update frightenedTime
	}
	//if (frightenedTime <= 0.0f) {
	if (mGhost->frightenedTime <= 0) { // frightenedTime has run out -> go back to scatter
		mState = Scatter;
		scatterTime = 5.0f;
		//Start(mNextNode);
		//ScatterUpdate(deltaTime);
		return;
		//mGhost->GetComponent<AnimatedSprite>()->SetAnimation("down"); // here for testing
		//frightenedTime == 0;
	}
	else if (mGhost->frightenedTime > 2.0f) {
		// First five seconds -> solid blue frightened animation
		mGhost->GetComponent<AnimatedSprite>()->SetAnimation("scared0");
	}
	else {
		// Last two seconds -> start flashing white and blue
		mGhost->GetComponent<AnimatedSprite>()->SetAnimation("scared1");
	}

	float ghostX = mGhost->GetPosition().x;
	float ghostY = mGhost->GetPosition().y;
	Vector2 gPos = Vector2(ghostX, ghostY);

	CollisionComponent* coll = mGhost->GetComponent<CollisionComponent>();

	CollSide cs = coll->GetMinOverlap(mNextNode->GetComponent<CollisionComponent>(), gPos);
	if (cs != CollSide::None) {
		// next node should be random adjacent node that isn't prev node
		// Make sure not first time when you must reversing path 
		//if (mNextNode != mPrevNode) {
		std::vector<PathNode*> neighbors = mNextNode->mAdjacent;
		PathNode* next = neighbors.at(rand() % neighbors.size());
		while (next == mPrevNode || next->GetType() == next->Ghost) {
			next = mNextNode->mAdjacent.at(rand() % neighbors.size());
		}
		mPrevNode = mNextNode;
		mNextNode = next;
		//}

		// Update direction:
		//if (!inTunnel) {
			if (mNextNode->GetPosition().x == mPrevNode->GetPosition().x) {
				if (mNextNode->GetPosition().y < mPrevNode->GetPosition().y) {
					(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("up");
					dir = "up";

				}
				else if (mNextNode->GetPosition().y > mPrevNode->GetPosition().y) {
					(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("down");
					dir = "down";
				}
			}
			else if (mNextNode->GetPosition().y == mPrevNode->GetPosition().y) {
				if (mNextNode->GetPosition().x <= mPrevNode->GetPosition().x) {
					(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("left");
					dir = "left";
				}
				if (mNextNode->GetPosition().x > mPrevNode->GetPosition().x) {
					(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("right");
					dir = "right";
				}
			}
		//}

		if (mNextNode->GetType() == PathNode::Tunnel) {
			if (dir == "left") {
				mNextNode = mGhost->mGame->mTunnelRight;
			}
			else if (dir == "right") {
				mNextNode = mGhost->mGame->mTunnelLeft;
			}
			mGhost->SetPosition(mNextNode->GetPosition());
		}
	}
	Vector2 pos = mOwner->GetPosition();
	if (dir == "up") {
		pos.y -= 65.0f * deltaTime;
	}
	else if (dir == "down") {
		pos.y += 65.0f * deltaTime;
	}
	else if (dir == "left") {
		pos.x -= 65.0f * deltaTime;
	}
	else if (dir == "right") {
		pos.x += 65.0f * deltaTime;
	}
	mGhost->SetPosition(pos);
}

void GhostAI::ScatterUpdate(float deltaTime) {
	/*
		Ghost has made it to mNextNode if there is a collision (both Ghost
		and PathNode have CollisionComponents). If you make it to mNextNode, you should:
			• Set the ghosts position to mNextNode
			• Set mPrevNode = mNextNode
			• Pop off the next node from mPath and save it as mNextNode
			• Change the movement direction of the ghost (if needed)
	*/

	//if (mNextNode != mPrevNode) {
	//if(mNextNode != nullptr){

	float ghostX = mGhost->GetPosition().x;
	float ghostY = mGhost->GetPosition().y;
	Vector2 gPos = Vector2(ghostX, ghostY);

	CollisionComponent* coll = mGhost->GetComponent<CollisionComponent>();

	CollSide cs = coll->GetMinOverlap(mNextNode->GetComponent<CollisionComponent>(), gPos);
	if (cs != CollSide::None) {
		bool isTunnel = false;
		// next node should be set to next in path and then removed from path
		if (!mPath.empty()) {
			if (mNextNode->GetType() == PathNode::Tunnel) {
				isTunnel = true;
				if (dir == "left") {
					//mNextNode = mGhost->mGame->mTunnelRight;
				}
				else if (dir == "right") {
					//mNextNode = mGhost->mGame->mTunnelLeft;
				}
				mGhost->SetPosition(mNextNode->GetPosition());

				mPrevNode = mPath.back();
				mPath.pop_back();
				mNextNode = mPath.back();
				mPath.pop_back();
			}
			else {
				mPrevNode = mNextNode;
				mNextNode = mPath.back();
				mPath.pop_back();
			}
			
		}
		// If path is empty (target node reached), need to come up with next node:
		else {
			// Clear path (though should be empty)
			mPath.clear();
			mTargetNode = mGhost->GetScatterNode(); //Set target to scatter node for now
	
			// If can make new valid path with A* to scatter node and follow that
			if (AStar(mNextNode, mTargetNode, mNextNode)) {
			}

			//If not, set mNextNode to neighboring PathNode that isn’t mPrevNode and is closest to goal
			else {
				// Set next node to the node in next's neighbors that is closest to goal
				std::vector<PathNode*> neighbors = mNextNode->mAdjacent;
				PathNode* closest = mNextNode->mAdjacent.at(0);
				if (closest == mPrevNode) {
					if (neighbors.size() <= 1) {
						Start(mGhost->GetSpawnNode());
						return;
					}
					else {
						closest = mNextNode->mAdjacent.at(1);
					}
				}

				for (auto adj : neighbors) {
					if ((mPrevNode->GetPosition() - adj->GetPosition()).Length() <
						(mPrevNode->GetPosition() - closest->GetPosition()).Length()
						&& adj != mPrevNode && adj->GetType() != adj->Ghost) {
						closest = adj;
					}
				}
				mPrevNode = mNextNode;
				mNextNode = closest;
			}
		}

		// Update direction:
		if (!isTunnel) {
			if (mNextNode->GetPosition().x == mPrevNode->GetPosition().x) {
				if (mNextNode->GetPosition().y < mPrevNode->GetPosition().y) {
					(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("up");
					dir = "up";

				}
				else if (mNextNode->GetPosition().y > mPrevNode->GetPosition().y) {
					(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("down");
					dir = "down";
				}
			}
			else if (mNextNode->GetPosition().y == mPrevNode->GetPosition().y) {
				if (mNextNode->GetPosition().x <= mPrevNode->GetPosition().x) {
					(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("left");
					dir = "left";
				}
				if (mNextNode->GetPosition().x > mPrevNode->GetPosition().x) {
					(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("right");
					dir = "right";
				}
			}
		}
	}
	Vector2 pos = mOwner->GetPosition();
	if (dir == "up") {
		pos.y -= 90.0f * deltaTime;
	}
	else if (dir == "down") {
		pos.y += 90.0f * deltaTime;
	}
	else if (dir == "left") {
		pos.x -= 90.0f * deltaTime;
	}
	else if (dir == "right") {
		pos.x += 90.0f * deltaTime;
	}
	mGhost->SetPosition(pos);
	//} 
}

void GhostAI::ChaseUpdate(float deltaTime) {
	if (mNextNode != nullptr) {
		float ghostX = mGhost->GetPosition().x;
		float ghostY = mGhost->GetPosition().y;
		Vector2 gPos = Vector2(ghostX, ghostY);

		CollisionComponent* coll = mGhost->GetComponent<CollisionComponent>();

		CollSide cs = coll->GetMinOverlap(mNextNode->GetComponent<CollisionComponent>(), gPos);
		if (cs != CollSide::None) {
			mPath.clear();
				if (mGhost->GetType() == mGhost->Blinky) {
					mTargetNode = mGhost->mGame->mPlayer->GetPrevNode();
				}
				else if (mGhost->GetType() == mGhost->Pinky) {
					Vector2 targetPos = mGhost->mGame->mPlayer->GetPointInFrontOf(80);
					PathNode* closest = mGhost->mGame->mPathNodes.at(0);
					if (closest == mPrevNode) {
						if (mGhost->mGame->mPathNodes.size() > 1) {
							closest = mGhost->mGame->mPathNodes.at(1);
						}
					}
					for (auto n : mGhost->mGame->mPathNodes) {
						if (n != mPrevNode && n->GetType() != n->Ghost) {
							if ((n->GetPosition() - targetPos).Length() < (closest->GetPosition() - targetPos).Length()) {
								closest = n;
							}
						}
					}
					mTargetNode = closest;
				}
				else if (mGhost->GetType() == mGhost->Inky) {
					Vector2 pointP = mGhost->mGame->mPlayer->GetPointInFrontOf(40);
					Vector2 InkyToTarget;
					Vector2 pointQ;
					for (auto g : mGhost->mGame->mGhosts) {
						if (g->GetType() == g->Blinky) {
							InkyToTarget = g->GetPosition() - pointP;
							pointQ = ((InkyToTarget) * 2) + g->GetPosition();
						}
					}

					PathNode* closest = mGhost->mGame->mPathNodes.at(0);
					if (closest == mPrevNode) {
						if (mGhost->mGame->mPathNodes.size() > 1) {
							closest = mGhost->mGame->mPathNodes.at(1);
						}
					}
					for (auto n : mGhost->mGame->mPathNodes) {
						if (n != mPrevNode && n->GetType() != n->Ghost) {
							if ((n->GetPosition() - pointQ).Length() < (closest->GetPosition()-pointQ).Length()) {
								closest = n;
							}
						}
					}
					mTargetNode = closest;
				}
				else if (mGhost->GetType() == mGhost->Clyde) {
					float dist = (mGhost->GetPosition() - mGhost->mGame->mPlayer->GetPosition()).Length();
					if (dist > 150.0f) {
						mTargetNode = mGhost->mGame->mPlayer->GetPrevNode();
					}
					else {
						mTargetNode = mGhost->GetScatterNode();
					}
				}

				// If can make new valid path with A* to scatter node and follow that
				if (AStar(mNextNode, mTargetNode, mNextNode)) {
				}

				//If not, set mNextNode to neighboring PathNode that isn’t mPrevNode and is closest to goal
				else {
					// Set next node to the node in next's neighbors that is closest to goal
					std::vector<PathNode*> neighbors = mNextNode->mAdjacent;
					PathNode* closest = mNextNode->mAdjacent.at(0);
					if (closest == mPrevNode) {
						if (neighbors.size() >= 2) {
							closest = mNextNode->mAdjacent.at(1);
						}
					}
					if (neighbors.size() >= 2) {
						for (auto adj : neighbors) {
							if ((mPrevNode->GetPosition() - adj->GetPosition()).Length() <
								(mPrevNode->GetPosition() - closest->GetPosition()).Length()
								&& adj != mPrevNode && adj->GetType() != adj->Ghost) {
								closest = adj;
							}
						}
						mPrevNode = mNextNode;
						mNextNode = closest;
					}
				}

			// Update direction:
			if (mNextNode->GetPosition().x == mPrevNode->GetPosition().x) {
				if (mNextNode->GetPosition().y < mPrevNode->GetPosition().y) {
					(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("up");
					dir = "up";

				}
				else if (mNextNode->GetPosition().y > mPrevNode->GetPosition().y) {
					(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("down");
					dir = "down";
				}
			}
			else if (mNextNode->GetPosition().y == mPrevNode->GetPosition().y) {
				if (mNextNode->GetPosition().x <= mPrevNode->GetPosition().x) {
					(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("left");
					dir = "left";
				}
				if (mNextNode->GetPosition().x > mPrevNode->GetPosition().x) {
					(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("right");
					dir = "right";
				}
			}

			if (mNextNode->GetType() == PathNode::Tunnel) {
				if (dir == "left") {
					mNextNode = mGhost->mGame->mTunnelRight;
				}
				else if (dir == "right") {
					mNextNode = mGhost->mGame->mTunnelLeft;
				}
				mGhost->SetPosition(mNextNode->GetPosition());
			}
		}
	}
	Vector2 pos = mOwner->GetPosition();
	if (dir == "up") {
		pos.y -= 90.0f * deltaTime;
	}
	else if (dir == "down") {
		pos.y += 90.0f * deltaTime;
	}
	else if (dir == "left") {
		pos.x -= 90.0f * deltaTime;
	}
	else if (dir == "right") {
		pos.x += 90.0f * deltaTime;
	}
	mGhost->SetPosition(pos);
	//} 
}

void GhostAI::DeadUpdate(float deltaTime) {
	//if (mNextNode != mPrevNode) {
	//if(mNextNode != nullptr){

	float ghostX = mGhost->GetPosition().x;
	float ghostY = mGhost->GetPosition().y;
	Vector2 gPos = Vector2(ghostX, ghostY);

	CollisionComponent* coll = mGhost->GetComponent<CollisionComponent>();

	CollSide cs = coll->GetMinOverlap(mNextNode->GetComponent<CollisionComponent>(), gPos);
	if (cs != CollSide::None) {
		// next node should be set to next in path and then removed from path
		if (!mPath.empty()) {
			mPrevNode = mNextNode;
			mNextNode = mPath.back();
			mPath.pop_back();
		}
		// If path is empty (target node reached), need to come up with next node:
		else {
			//mState = Scatter;
			//scatterTime = 5.0f;
			mPath.clear();
			Start(mGhost->GetSpawnNode());
			//ScatterUpdate(deltaTime);
			return;
		}
		// Update direction:
		if (mNextNode->GetPosition().x == mPrevNode->GetPosition().x) {
			if (mNextNode->GetPosition().y < mPrevNode->GetPosition().y) {
				(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("deadup");
				dir = "up";

			}
			else if (mNextNode->GetPosition().y > mPrevNode->GetPosition().y) {
				(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("deaddown");
				dir = "down";
			}
		}
		else if (mNextNode->GetPosition().y == mPrevNode->GetPosition().y) {
			if (mNextNode->GetPosition().x <= mPrevNode->GetPosition().x) {
				(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("deadleft");
				dir = "left";
			}
			if (mNextNode->GetPosition().x > mPrevNode->GetPosition().x) {
				(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("deadright");
				dir = "right";
			}
		}
	}
	Vector2 pos = mOwner->GetPosition();
	if (dir == "up") {
		pos.y -= 125.0f * deltaTime;
	}
	else if (dir == "down") {
		pos.y += 125.0f * deltaTime;
	}
	else if (dir == "left") {
		pos.x -= 125.0f * deltaTime;
	}
	else if (dir == "right") {
		pos.x += 125.0f * deltaTime;
	}
	mGhost->SetPosition(pos);
	//} 
}

void GhostAI::Update(float deltaTime)
{
	if (mState == Frightened) {
		mTargetNode = nullptr;
		FrightenedUpdate(deltaTime);
	}
	else if (mState == Scatter) {
		if (scatterTime > 0.0f) {
			scatterTime -= deltaTime;
			ScatterUpdate(deltaTime);
		}
		else {
			mPath.clear();
			chaseTime = 20.0f;
			mState = Chase;
		}
	}
	else if (mState == Dead) {
		//mTargetNode = mGhost->GetSpawnNode();
		DeadUpdate(deltaTime);
	}
	else if (mState == Chase) {
		if (chaseTime > 0.0f) {
			chaseTime -= deltaTime;
			ChaseUpdate(deltaTime);
		}
		else {
			mPath.clear();
			scatterTime = 5.0f;
			mState = Scatter;
		}
	}
}


void GhostAI::Frighten()
{
	mState = Frightened;
	mPath.clear();
	mGhost->frightenedTime = 7.0f;
	if (!(mGhost->IsDead())) {
		mNextNode = mPrevNode;
	}

	// Update direction:
	if (dir == "up") {	
		(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("down");
		dir = "down";
	}
	else if (dir == "down") {
		(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("up");
		dir = "up";
	}
	else if (dir == "left") {
		(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("right");
		dir = "right";
	}
	else if (dir == "right") {
		(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("left");
		dir = "left";
	}
}

void GhostAI::Start(PathNode* startNode)
{
	/*
		• Set position of owner to position of startNode
		• Set mState to Scatter
		• Set mPrevNode, mNextNode, and mTargetNode to nullptr
		• Setup path from startNode to Ghost’s scatter node(mGhost->GetScatterNode())
			 -> call function to calculate the path using A*
	*/

	mOwner->SetPosition(mGhost->GetSpawnNode()->GetPosition());
	mState = Scatter;
	mPrevNode = nullptr;
	mNextNode = nullptr;
	mTargetNode = nullptr;

	scatterTime = 5.0f;
	mPath.clear();
	
	dir = "up";
	if (mGhost->GetType() == mGhost->Inky) {
		mGhost->SetPosition(Vector2(mGhost->GetPosition().x + 13, mGhost->GetPosition().y));
	}
	else if (mGhost->GetType() == mGhost->Clyde) {
		mGhost->SetPosition(Vector2(mGhost->GetPosition().x - 13, mGhost->GetPosition().y));
	}
	AStar(startNode, mGhost->GetScatterNode(), startNode);
}

float GhostAI::computeG(PathNode* x, PathNode* p, std::unordered_map<PathNode*, float> mG) {
	//edge cost b/w two adjacent path nodes is Euclidean distance, except when both in Tunnel (cost=0)
	
	float edgeCost = 0;
	if (x->GetType() == PathNode::Tunnel && p->GetType() == PathNode::Tunnel) {
		edgeCost = 0;
	}
	else {
		Vector2 dist = Vector2(x->GetPosition() - p->GetPosition());
		edgeCost = dist.Length();
	}
	
	// g(x) is the parent’s g plus whatever the cost is of the edge between the parent and x
		// g(x) = g(p) + EdgeCost(p, x)
	float g = mG[p] + edgeCost;

	return g;
}

float GhostAI::computeH(PathNode* x, PathNode* goal, std::unordered_map<PathNode*, float> mH) {
	//Ghosts can use PathNodes of all three types(Default, Ghost, and Tunnel). 
	//Each PathNode has a vector of pointers to its adjacent PathNodes.

	/* HEURISTIC: Euclidean distance
		However, because of teleporting tunnels, to guarantee h(x) is admissible, need to return the min of:
			o Euclidean distance from x to goal
			o Euclidean distance from x to tunnelLeft + Euclidean distance from tunnelRight to goal
			o Euclidean distance from x to tunnelRight + Euclidean distance from tunnelLeft to goal
	*/
	Vector2 dist = Vector2(x->GetPosition() - goal->GetPosition());
	float euc1 = dist.Length();

	Vector2 RightToGoal = Vector2((mOwner->GetGame()->mTunnelRight->GetPosition() - goal->GetPosition()));
	Vector2 xToLeft = Vector2(x->GetPosition() - mOwner->GetGame()->mTunnelLeft->GetPosition());
	float euc2 = RightToGoal.Length() + xToLeft.Length();

	Vector2 LeftToGoal = Vector2((mOwner->GetGame()->mTunnelLeft->GetPosition() - goal->GetPosition()));
	Vector2 xToRight = Vector2(x->GetPosition() - mOwner->GetGame()->mTunnelRight->GetPosition());
	float euc3 = LeftToGoal.Length() + xToRight.Length();

	float h = fmin(euc1, euc2);
	h = fmin(euc2, euc3);

	return h;
}

bool GhostAI::AStar(PathNode* startNode, PathNode* goalNode, PathNode* currentNode) {

	std::vector<PathNode*> open;
	std::unordered_map<PathNode*, float> mF;
	std::unordered_map<PathNode*, float> mH;
	std::unordered_map<PathNode*, float> mG;
	std::unordered_map<PathNode*, bool> inClosed;
	std::unordered_map<PathNode*, PathNode*> mParents;

	mPath.clear();
	currentNode = startNode;
	inClosed[currentNode] = 1; //add currentNode to closedSet

	do {
		for (auto node : (currentNode->mAdjacent)) {
			
			auto iter = std::find(open.begin(), open.end(), node);
			if (inClosed[node] == 1) {
				continue;
			}
			else if (node == mPrevNode) {
				continue; // * don’t add mPrevNode to the open set
			}
			else if (iter != open.end()) {
				float newG = computeG(node, currentNode, mG);
				if (newG < mG[node]) {
					mParents[node] = currentNode;
					mG[node] = newG;
					mF[node] = mG[node] + mH[node];
				}
			}
			else {
					mParents[node] = currentNode;
					float newH = computeH(node, goalNode, mH);
					float newG = computeG(node, currentNode, mG);
					mH[node] = newH;
					mG[node] = newG;
					mF[node] = mH[node] + mG[node];
					open.emplace_back(node);
			}
		}
		

		if (open.empty()) {
			break;
		}

		// select the lowest f(x) cost node in the open set
			//float f = h + g;
		// currentNode = Node with lowest f in openSet

		currentNode = open.at(0);
		for (auto node : open) {
			if (mF[node] < mF[currentNode]) {
				currentNode = node;
			}
		}

		//move currentNode from openSet to closedSet:
		inClosed[currentNode] = 1;

		auto iter = std::find(open.begin(), open.end(), currentNode);
		if (iter != open.end())
		{
			// Swap to end of vector and pop off (avoid erase copies)
			auto iter2 = open.end() - 1;
			std::iter_swap(iter, iter2);
			open.pop_back();
		}

	} while (currentNode != goalNode);

	/* AFTER SEARCH:
		• mTargetNode should be goal node
		• mPrevNode should be start node
		• mNextNode should contain node on the path that’s after start node
		• The mPath vector should contain every node on the path that’s after mNextNode (it’s
			possible this is empty if mNextNode is the goal)
	*/

	mTargetNode = goalNode;

	if (currentNode == goalNode) {
		mPrevNode = startNode;
		PathNode* cNode = goalNode;
		while (cNode != nullptr) {
			//mPath.emplace_back(cNode);
			mPath.emplace_back(cNode);
			cNode = mParents[cNode];
		}
		mPath.pop_back();
		mNextNode = mPath.back();
		mPath.pop_back();
		return 1;
	}

	else {
		while (!mPath.empty()) {
			mPath.pop_back();
		}
		//mPath.clear();
		return 0;
	}
}

void GhostAI::Die()
{
	
	mPath.clear();
	//mOwner->SetPosition(mPrevNode->GetPosition());
	AStar(mPrevNode, mGhost->GetSpawnNode(), mPrevNode);
	// Update direction:
	if (mNextNode->GetPosition().x == mPrevNode->GetPosition().x) {
		if (mNextNode->GetPosition().y < mPrevNode->GetPosition().y) {
			(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("deadup");
			dir = "up";

		}
		else if (mNextNode->GetPosition().y > mPrevNode->GetPosition().y) {
			(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("deaddown");
			dir = "down";
		}
	}
	else if (mNextNode->GetPosition().y == mPrevNode->GetPosition().y) {
		if (mNextNode->GetPosition().x <= mPrevNode->GetPosition().x) {
			(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("deadleft");
			dir = "left";
		}
		if (mNextNode->GetPosition().x > mPrevNode->GetPosition().x) {
			(mGhost->GetComponent<AnimatedSprite>())->SetAnimation("deadright");
			dir = "right";
		}
	}
	mGhost->SetPosition(mPrevNode->GetPosition());
	mState = Dead;
}

void GhostAI::DebugDrawPath(SDL_Renderer* render)
{
	// Draw a rectangle at the target node
	if (mTargetNode != nullptr)
	{
		const int SIZE = 16;
		SDL_Rect r;
		r.x = static_cast<int>(mTargetNode->GetPosition().x) - SIZE / 2;
		r.y = static_cast<int>(mTargetNode->GetPosition().y) - SIZE / 2;
		r.w = SIZE;
		r.h = SIZE;
		SDL_RenderDrawRect(render, &r);
	}

	// Line from ghost to next node
	if (mNextNode != nullptr)
	{
		SDL_RenderDrawLine(render,
			static_cast<int>(mOwner->GetPosition().x),
			static_cast<int>(mOwner->GetPosition().y),
			static_cast<int>(mNextNode->GetPosition().x),
			static_cast<int>(mNextNode->GetPosition().y));
	}

	// Exit if no path
	if (mPath.empty())
	{
		return;
	}

	// Line from next node to subsequent on path
	SDL_RenderDrawLine(render,
		static_cast<int>(mNextNode->GetPosition().x),
		static_cast<int>(mNextNode->GetPosition().y),
		static_cast<int>(mPath.back()->GetPosition().x),
		static_cast<int>(mPath.back()->GetPosition().y));

	// Lines for rest of path
	for (size_t i = 0; i < mPath.size() - 1; i++)
	{
		SDL_RenderDrawLine(render,
			static_cast<int>(mPath[i]->GetPosition().x),
			static_cast<int>(mPath[i]->GetPosition().y),
			static_cast<int>(mPath[i + 1]->GetPosition().x),
			static_cast<int>(mPath[i + 1]->GetPosition().y));
	}
}
