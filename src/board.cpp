#include "board.h"
#include <queue>
#include <cfloat>

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


// A*の探索ノード（f値が小さい順に優先度付きキューから取り出される）
struct Node {
	Point pos;
	float g;// 始点からの実コスト
	float f;// g + ヒューリスティック(終点までの予想コスト)

	bool operator > (const Node& rhs) const { return f > rhs.f; }
};

bool Board::find(const Point& 始点, const Point& 終点, std::vector<std::vector<Mass>> &mass) const
{
	mass[始点.y][始点.x].set(Mass::START);
	mass[終点.y][終点.x].set(Mass::GOAL);

	// オープンリスト（f値の昇順で取り出す）
	std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

	// 各マスへの現時点での最小コスト(g値)
	std::vector<std::vector<float>> g値(mass.size(), std::vector<float>(mass[0].size(), FLT_MAX));

	g値[始点.y][始点.x] = 0.0f;
	open.push({ 始点, 0.0f, Point::distance(始点, 終点) });

	while (!open.empty()) {
		Node 現在 = open.top(); open.pop();

		// より安い経路で更新済みの古いノードは読み捨てる
		if (現在.g > g値[現在.pos.y][現在.pos.x]) { continue; }

		// 終点に到達したら親を辿って経路を復元する
		if (現在.pos == 終点) {
			Point p = mass[終点.y][終点.x].getParent();
			while (p != 始点) {
				mass[p.y][p.x].set(Mass::WAYPOINT);
				p = mass[p.y][p.x].getParent();
			}
			return true;
		}

		// 4方向に移動できるか
		const Point 方向[] = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } };
		for (const Point& d : 方向) {
			Point 次 = 現在.pos + d;
			if (!map_[次.y][次.x].canMove()) { continue; }// 壁(#)は通れない

			// 移動コスト = 次のマスのコスト(WATERは3倍遅い、ROADは3倍速い)
			float g = 現在.g + map_[次.y][次.x].getCost();
			if (g値[次.y][次.x] <= g) { continue; }// 既により安い経路がある

			g値[次.y][次.x] = g;
			mass[次.y][次.x].visit(現在.pos);// 経路復元用に親を記録
			open.push({ 次, g, g + Point::distance(次, 終点) });
		}
	}
	return false;// 経路が見つからなかった
}
