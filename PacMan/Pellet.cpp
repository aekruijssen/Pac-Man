#include "Pellet.h"
#include "SpriteComponent.h"
#include "Game.h"
#include "CollisionComponent.h"
#include "Collectible.h"

Pellet::Pellet(class Game* game)
:Actor(game)
{
	SpriteComponent* sc = new SpriteComponent(this);
	sc->SetTexture(game->GetTexture("Assets/Pellet.png"));
	
	CollisionComponent* cc = new CollisionComponent(this);
	cc->SetSize(2.0f, 2.0f);
	game->mPelletCount++;
	
	new Collectible(this, [this]{
		mGame->mPelletCount--;
	});
}
