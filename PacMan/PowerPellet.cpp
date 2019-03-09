#include "PowerPellet.h"
#include "SpriteComponent.h"
#include "Game.h"
#include "CollisionComponent.h"
#include "Collectible.h"
#include "Ghost.h"

PowerPellet::PowerPellet(class Game* game)
:Actor(game)
{
	SpriteComponent* sc = new SpriteComponent(this);
	sc->SetTexture(game->GetTexture("Assets/PowerPellet.png"));
	
	CollisionComponent* cc = new CollisionComponent(this);
	cc->SetSize(2.0f, 2.0f);
	
	new Collectible(this, [this]{
		for (auto g : mGame->mGhosts)
		{
			g->frightenedTime = 7.0f;
			g->Frighten();
		}
	});
}
