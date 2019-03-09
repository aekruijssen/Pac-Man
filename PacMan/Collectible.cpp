#include "Collectible.h"
#include "Actor.h"
#include "Game.h"
#include "PacMan.h"
#include "CollisionComponent.h"

Collectible::Collectible(Actor* owner, std::function<void()> onCollect)
:Component(owner, 50)
,mOnCollect(onCollect)
{
	
}

void Collectible::Update(float deltaTime)
{
	if (mOwner->GetGame()->mPlayer->GetComponent<CollisionComponent>()->
		Intersect(mOwner->GetComponent<CollisionComponent>()))
	{
		mOwner->SetState(ActorState::Destroy);
		mOnCollect();
	}
}
