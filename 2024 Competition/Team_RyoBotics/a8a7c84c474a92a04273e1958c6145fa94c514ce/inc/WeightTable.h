#pragma once
#include <string>
#include <memory>
#include <vector>
#include "CommonTypes.h"

// WeightTable クラスは、各セルに対して5種類（右, 下, 左, 上, 待機）の重みを管理します。
class WeightTable
{
public:
    // コンストラクタ: マップの行数と列数を渡してインスタンスを生成
    WeightTable(int rows, int cols);

    // ファイルから重みを読み込み、読み込んだファイルのサフィックス（stem）を返す関数
    std::string loadWeights(const std::string &weights_path);

    // 重みを取得
    WeightedCostType getWeight(int loc, int from_dir) const;

private:
    int rows_;
    int cols_;
    // 各セルに5種類の重み (右, 下, 左, 上, 待機) を格納する。初期値は全て 1.0f です。
    std::shared_ptr<std::vector<WeightedCostType>> weights_;
};