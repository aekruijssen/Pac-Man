#pragma once
#include "Component.h"
#include <functional>

class Collectible : public Component
{
public:
	Collectible(class Actor* owner, std::function<void()> onCollect);
	void Update(float deltaTime) override;
private:
	std::function<void()> mOnCollect;
};
