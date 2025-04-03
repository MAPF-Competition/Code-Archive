#include <gtest/gtest.h>
#include "distance_table.h"
#include "SharedEnv.h"
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class DistanceTableTest : public ::testing::Test
{
protected:
    SharedEnvironment *env;

    void SetUp() override
    {
        // テスト用の小さなマップを作成
        env = new SharedEnvironment();
        env->rows = 4;
        env->cols = 4;
        env->map = {
            0, 0, 0, 0,
            0, 1, 0, 0, // 1は障害物
            0, 0, 1, 0,
            0, 0, 0, 0};
    }

    void TearDown() override
    {
        delete env;
    }

    // 手動で計算した最短距離と比較する補助関数
    void verifyDistance(const DefaultPlanner::FastDistanceTable &table, int from, int to, int expected)
    {
        int actual = table.getDistance(from, to);
        EXPECT_EQ(actual, expected)
            << "Distance from " << from << " to " << to
            << ": expected " << expected << " but got " << actual;
    }
};

TEST_F(DistanceTableTest, BasicDistanceCheck)
{
    auto table = DefaultPlanner::DistanceTable::computeAllPairsShortestPaths(env);

    // 隣接セル間の距離は1になるはず
    verifyDistance(table, 0, 1, 1); // 横方向
    verifyDistance(table, 0, 4, 1); // 縦方向

    // 障害物を迂回する必要がある場合
    verifyDistance(table, 0, 2, 2); // 障害物なしの直線経路
    verifyDistance(table, 4, 6, 3); // 障害物を迂回

    // 対角線上のセル間
    verifyDistance(table, 0, 5, 2);  // 障害物なし
    verifyDistance(table, 5, 10, 3); // 障害物を迂回
}

TEST_F(DistanceTableTest, ObstacleCheck)
{
    auto table = DefaultPlanner::DistanceTable::computeAllPairsShortestPaths(env);

    // 障害物セルへの/からの距離はINT_MAXになるはず
    verifyDistance(table, 0, 5, std::numeric_limits<int>::max()); // 障害物セルへ
    verifyDistance(table, 5, 0, std::numeric_limits<int>::max()); // 障害物セルから
}

TEST_F(DistanceTableTest, SymmetryCheck)
{
    auto table = DefaultPlanner::DistanceTable::computeAllPairsShortestPaths(env);

    // 任意の2点間の距離は双方向で同じになるはず
    for (int i = 0; i < env->map.size(); i++)
    {
        for (int j = 0; j < env->map.size(); j++)
        {
            EXPECT_EQ(table.getDistance(i, j), table.getDistance(j, i))
                << "Asymmetric distance between " << i << " and " << j;
        }
    }
}

TEST_F(DistanceTableTest, SaveAndLoadCheck)
{
    auto original_table = DefaultPlanner::DistanceTable::computeAllPairsShortestPaths(env);

    // 一時ファイルに保存
    std::string temp_file = "test_distance_table.bin";
    {
        std::ofstream file(temp_file, std::ios::binary);
        ASSERT_TRUE(file.good()) << "Failed to open file for writing";
        original_table.save(file);
    }

    // 読み込んで比較
    auto loaded_table = DefaultPlanner::DistanceTable::load(temp_file);

    // すべてのペアで距離を比較
    for (int i = 0; i < env->map.size(); i++)
    {
        for (int j = 0; j < env->map.size(); j++)
        {
            EXPECT_EQ(original_table.getDistance(i, j), loaded_table.getDistance(i, j))
                << "Mismatch in loaded distance between " << i << " and " << j;
        }
    }

    // テスト用ファイルを削除
    std::remove(temp_file.c_str());
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}