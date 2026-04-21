#include "board.h"

std::map<Mass::status, MassInfo> Mass::statusData =
{
	{ BLANK, { 1.0f, ' '}},
	{ WALL,  {-1.0f, '#'}},
	{ WATER, { 3.0f, '~'}},
	{ ROAD,  { 0.3f, '$'}},

	// 動的な要素
	{ START,	{-1.0f, 'S'}},
	{ GOAL,		{-1.0f, 'G'}},
	{ WAYPOINT, {-1.0f, 'o'}},

	{ INVALID,  {-1.0f, '\0'}},
};


bool Board::find(const Point& 始点, const Point& 終点, std::vector<std::vector<Mass>> &mass) const
{
	mass[始点.y][始点.x].set(Mass::START);
	mass[終点.y][終点.x].set(Mass::GOAL);

	// 経路探索
	Point 現在 = 始点;
	while (現在 != 終点) {
		// 歩いた場所に印をつける(見やすさのために始点は書き換えない)
		if (現在 != 始点){mass[現在.y][現在.x].set(Mass::WAYPOINT);}

		// ★★★ todo: ここを素敵にしよう
		// 終点に向かって歩く
		int dx = 終点.x - 現在.x;
		int dy = 終点.y - 現在.y;

		Point 左右 = 現在; 左右.x += (dx > 0) ? 1 : -1;
		Point 上下 = 現在; 上下.y += (dy > 0) ? 1 : -1;

		if (dx * dx < dy * dy)
		{
			// Y方向に近づこうとして、だめならX方向に動く
			if (map_[上下.y][上下.x].canMove()) { 現在 = 上下; continue; }
			if (map_[左右.y][左右.x].canMove()) { 現在 = 左右; continue; }
		}

		else
		{
			// X方向に近づこうとして、だめならY方向に動く
			if (map_[左右.y][左右.x].canMove()) { 現在 = 左右; continue; }
			if (map_[上下.y][上下.x].canMove()) { 現在 = 上下; continue; }
		}
	}

	return true;
}
